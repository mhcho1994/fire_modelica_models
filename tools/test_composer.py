"""Minimum NED/FRD frontend and artifact behavior: python3 -m unittest discover -s tools -p test_composer.py."""
import json
import hashlib
import math
import os
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BIN = ROOT / 'tools/fire-compose/target/debug/fire-compose'


class RustComposerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        subprocess.run(['cargo', 'build', '--locked', '--offline', '--manifest-path',
                        str(ROOT / 'tools/fire-compose/Cargo.toml')], check=True)

    def run_config(self, text, action='validate', *args):
        with tempfile.TemporaryDirectory() as work:
            config = Path(work) / 'vehicle.toml'
            config.write_text(text)
            return subprocess.run([str(BIN), action, str(config), *args],
                                  capture_output=True, text=True)

    def assert_config_equal(self, a, b):
        if isinstance(a, dict):
            self.assertEqual(set(a), set(b))
            for key in a:
                self.assert_config_equal(a[key], b[key])
        elif isinstance(a, list):
            self.assertEqual(len(a), len(b))
            for left, right in zip(a, b):
                self.assert_config_equal(left, right)
        elif isinstance(a, (int, float)) and not isinstance(a, bool):
            self.assertTrue(math.isclose(a, b, rel_tol=1e-14, abs_tol=1e-15), (a,b))
        else:
            self.assertEqual(a, b)

    def test_minimal_examples_resolve_ned_frd(self):
        for source in sorted((ROOT / 'configs').glob('*.toml')):
            with self.subTest(source=source.name):
                run = self.run_config(source.read_text(), 'validate', '--json')
                self.assertEqual(run.returncode, 0, run.stderr)
                resolved = json.loads(run.stdout)
                self.assertEqual(resolved['schema_version'], 3)
                self.assertEqual(resolved['acquisition'], 'continuous')
                self.assertLess(resolved['initial']['p_start'][2], 0)
                self.assertEqual(resolved['geometry']['R_br'][0], [[1,0,0],[0,-1,0],[0,0,-1]])
                self.assertLess(resolved['geometry']['armMount'][0][1], 0)
                self.assertNotIn('nLegs', resolved['geometry'])
                self.assertTrue(resolved['landing_gear']['enabled'])
                self.assertEqual(len(resolved['landing_gear']['position_C']), 4)

    def test_nested_physics_table_matches_standalone_configuration(self):
        standalone = ('schema_version=3\nmodel_name="NestedHexa"\nacquisition="continuous"\n'
                      '[geometry]\npreset="HexaX"\nnActuators=6\n')
        integrated = ('[Machine]\nplatform="STM32F427"\n'
                      '[FMU.models."hexa.variant".physics]\n'
                      + standalone.replace('[geometry]', '[FMU.models."hexa.variant".physics.geometry]'))
        selector = 'FMU.models."hexa.variant".physics'
        original = self.run_config(standalone, 'validate', '--json')
        selected = self.run_config(integrated, 'validate', '--config-table', selector, '--json')
        self.assertEqual(original.returncode, 0, original.stderr)
        self.assertEqual(selected.returncode, 0, selected.stderr)
        self.assert_config_equal(json.loads(original.stdout), json.loads(selected.stdout))
        with tempfile.TemporaryDirectory() as work:
            for name, raw, extra in [('standalone', standalone, ()),
                                     ('integrated', integrated, ('--config-table', selector))]:
                run = self.run_config(raw, 'emit', '--profile', 'fastdyn',
                                      '--output-dir', str(Path(work) / name), *extra)
                self.assertEqual(run.returncode, 0, run.stderr)
            expected = (Path(work) / 'standalone/NestedHexa.mo').read_bytes()
            self.assertEqual((Path(work) / 'integrated/NestedHexa.mo').read_bytes(), expected)

    def test_invalid_table_selection_does_not_emit(self):
        valid = '[physics]\nschema_version=3\nmodel_name="SelectedQuad"\n'
        cases = [(valid, 'missing'), (valid, 'physics.schema_version'), (valid, ''),
                 (valid, 'physics]\n[other'),
                 (valid + 'unknown_field=1\n', 'physics'),
                 (valid + '[physics.motor]\ntau=-1\n', 'physics')]
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for raw, selector in cases:
                with self.subTest(raw=raw, selector=selector):
                    run = self.run_config(raw, 'emit', '--config-table', selector,
                                          '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())

    def test_invalid_configuration_does_not_emit(self):
        cases = [
            'schema_version=true',
            'schema_version=1',
            'schema_version=2',
            'schema_version=3\nacquisition="sampled"',
            'schema_version=3\n[geometry]\nnLegs=0',
            'schema_version=3\n[ground]\nlegStiffness=1500',
            'schema_version=3\n[landing_gear]\nenabled=1',
            'schema_version=3\n[landing_gear]\nstiffness=-1',
            'schema_version=3\n[landing_gear]\ndamping=nan',
            'schema_version=3\n[landing_gear]\ntangentialDamping=-1',
            'schema_version=3\n[landing_gear]\nfrictionCoefficient=-1',
            'schema_version=3\n[landing_gear]\ngroundZ=inf',
            'schema_version=3\n[landing_gear]\nposition_C=[[0,0,0.1]]',
            'schema_version=3\n[landing_gear]\nnLegs=6',
            'schema_version=3\nacquisition=false',
            'schema_version=3\nacquisition="unknown"',
            'schema_version=3\nmodel_name="Bad; end Bad"',
            'schema_version=3\n[motor]\ntau=nan',
            'schema_version=3\n[motor]\ntau=true',
            'schema_version=3\n[geometry]\nactuatorIndex=[1,2,3,5]',
            'schema_version=3\n[mass.aggregate]\ninertia=[[1,0,0],[0,1,0],[0,0,3]]',
            'schema_version=3\n[models.imu]\nresponse="first_order"\ntau=[0.1,0.1]',
        ]
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for raw in cases:
                with self.subTest(raw=raw):
                    run = self.run_config(raw, 'emit', '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())

    def test_artifacts_contain_one_plant_and_track_acquisition(self):
        raw = 'schema_version=3\nmodel_name="TestHexa"\nacquisition="continuous"\n[geometry]\npreset="HexaX"\nnActuators=8\nactuatorIndex=[1,2,3,4,6,8]\n'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(output))
            self.assertEqual(run.returncode, 0, run.stderr)
            source = (output / 'TestHexa.mo').read_text()
            self.assertEqual(source.count('MultirotorWithSensors plant('), 1)
            self.assertNotIn('sampledSensors', source)
            self.assertNotIn('sampledActuators', source)
            manifest = json.loads((output / 'TestHexa.manifest.json').read_text())
            self.assertEqual(manifest['interface']['nActuators'], 8)
            self.assertEqual(manifest['event_requirements'],
                             {'ground_contact': True, 'sampled_actuators': False, 'sampled_sensors': False})
            self.assertEqual(manifest['physics']['contact'], 'four_point_spring_damper')
            self.assertEqual(manifest['core_contract'], 'ned_frd_continuous_v1')
            self.assertEqual(manifest['frames']['body'], 'FRD')
            self.assertFalse(any(k.startswith('deprecated/') for k in manifest['source']['files']))
            self.assertFalse((output / 'sources/fire_modelica_models/deprecated').exists())
            self.assertEqual(manifest['verification']['fmu_simulation'], 'not_run')
            self.assertTrue((output / 'sources/fire_modelica_models/package.mo').is_file())
            manifest['verification']['modelica_check'] = 'test-result'
            (output / 'TestHexa.manifest.json').write_text(json.dumps(manifest))
            stale = output / 'sources/fire_modelica_models/RemovedModel.mo'
            stale.write_text('model RemovedModel end RemovedModel;')
            repeat = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(output))
            self.assertEqual(repeat.returncode, 0, repeat.stderr)
            self.assertFalse(stale.exists())
            manifest = json.loads((output / 'TestHexa.manifest.json').read_text())
            self.assertEqual(manifest['verification']['modelica_check'], 'test-result')

    def test_contact_can_be_disabled_explicitly(self):
        raw = 'schema_version=3\nmodel_name="FreeVehicle"\n[landing_gear]\nenabled=false\n'
        with tempfile.TemporaryDirectory() as work:
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            manifest = json.loads((Path(work)/'FreeVehicle.manifest.json').read_text())
            self.assertFalse(manifest['event_requirements']['ground_contact'])
            self.assertEqual(manifest['physics']['contact'], 'none')
            self.assertFalse(manifest['config']['landing_gear']['enabled'])

    def test_mass_ids_stay_in_manifest_and_do_not_change_physics_source(self):
        aggregate = (ROOT / 'configs/quad.toml').read_text().replace(
            '[mass.aggregate]', '[mass.aggregate]\ncomponentId="vehicle.aggregate"')
        assembled = (ROOT / 'configs/payload.toml').read_text() + (
            '\n[[mass.additional_parts]]\ncomponentId="battery"\nmass=0.1\n'
            'inertia=[[0.001,0,0],[0,0.001,0],[0,0,0.001]]\n')
        for raw, model, expected in [
            (aggregate, 'ConfiguredQuad', ['vehicle.aggregate']),
            (assembled, 'ConfiguredPayload',
             ['frame.core', 'arm.1', 'arm.2', 'arm.3', 'arm.4', 'payload.camera', 'battery']),
        ]:
            for profile in ('plant', 'fastdyn'):
                with self.subTest(model=model, profile=profile), tempfile.TemporaryDirectory() as work:
                    output = Path(work)
                    run = self.run_config(raw, 'emit', '--profile', profile, '--output-dir', work)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    source = (output / f'{model}.mo').read_text()
                    self.assertNotIn('componentId', source)
                    manifest = json.loads((output / f'{model}.manifest.json').read_text())
                    mass = manifest['config']['mass']
                    parts = ([mass['aggregate']] if mass['mode'] == 'aggregate' else
                             [mass['core'], *mass['arms'], *mass['payloads'], *mass['additional_parts']])
                    self.assertEqual([p['componentId'] for p in parts], expected)
                    renamed = raw
                    for identity in expected:
                        renamed = renamed.replace(f'"{identity}"', f'"renamed.{identity}"')
                    run = self.run_config(renamed, 'emit', '--profile', profile, '--output-dir', work)
                    self.assertEqual(run.returncode, 0, run.stderr)
                    self.assertEqual((output / f'{model}.mo').read_text(), source)
                    updated = json.loads((output / f'{model}.manifest.json').read_text())
                    self.assertNotEqual(updated['config_sha256'], manifest['config_sha256'])

    def test_duplicate_mass_ids_are_rejected_before_emission(self):
        raw = (ROOT / 'configs/payload.toml').read_text()
        duplicates = [
            raw.replace('"payload.camera"', '"frame.core"'),
            raw.replace('"arm.2"', '"arm.1"'),
            raw.replace('"frame.core"', '"arm.1"'),
            raw + ('\n[[mass.additional_parts]]\ncomponentId="payload.camera"\nmass=0.1\n'
                   'inertia=[[0.001,0,0],[0,0.001,0],[0,0,0.001]]\n'),
        ]
        for index, duplicate in enumerate(duplicates):
            with self.subTest(collision=index), tempfile.TemporaryDirectory() as work:
                output = Path(work) / 'generated'
                run = self.run_config(duplicate, 'emit', '--output-dir', str(output))
                self.assertNotEqual(run.returncode, 0)
                self.assertIn('duplicate nonempty componentId', run.stderr)
                self.assertFalse(output.exists())
        for identity in ['frame.core', 'arm.1', 'arm.2', 'arm.3', 'arm.4', 'payload.camera']:
            raw = raw.replace(f'"{identity}"', '""')
        run = self.run_config(raw, 'validate')
        self.assertEqual(run.returncode, 0, run.stderr)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_assembled_mass_preserves_mass_and_cg(self):
        raw = (ROOT / 'configs/payload.toml').read_text()
        with tempfile.TemporaryDirectory() as work:
            work = Path(work)
            run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(work/'generated'))
            self.assertEqual(run.returncode, 0, run.stderr)
            harness = work/'MassRun.mo'
            harness.write_text('''model MassRun
  extends ConfiguredPayload(pwm=fill(1100,4));
initial equation
  assert(abs(plant.chassis.mass-1.7)<1e-12,"Composed assembled mass changed");
  assert(max(abs(plant.chassis.cg_C-{0.018/1.7,0,0.024/1.7}))<1e-12,
    "Composed assembled CG changed");
end MassRun;
''')
            files = [work/'generated/sources/fire_modelica_models/package.mo',
                     work/'generated/ConfiguredPayload.mo', harness]
            script = work/'simulate.mos'
            script.write_text('loadModel(Modelica,{"4.0.0"});\n'
                + ''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                + 'simulate(MassRun,stopTime=0.01);\ngetErrorString();\n')
            run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
            self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_ground_support_and_takeoff(self):
        for preset, count in [('QuadX', 4), ('HexaX', 6)]:
            with self.subTest(preset=preset), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                raw = (f'schema_version=3\nmodel_name="GroundVehicle"\n[geometry]\npreset="{preset}"\n'
                       '[landing_gear]\nenabled=true\ngroundZ=0.4\nstiffness=2400\ndamping=120\n'
                       'position_C=[[0.17,0.17,0.15],[-0.17,-0.17,0.15],'
                       '[0.17,-0.17,0.15],[-0.17,0.17,0.15]]\n'
                       '[initial]\np_start=[0,0,0.23]\n')
                run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                thrust_pwm = 1100 + 800*math.sqrt(1.3*1.5*9.80665/(count*1e-5))/1000
                harness = work/'GroundRun.mo'
                harness.write_text(f'''model GroundRun
  extends GroundVehicle(pwm=fill(if time<2 then 1100 else {thrust_pwm},{count}));
equation
  when time>=1.5 then
    assert(abs(truth.p_w[3]-(0.4-0.15+1.5*9.80665/9600))<1e-6,"Composed ground height/compression failed");
    assert(abs(accel[3]+9.80665)<1e-4,"Composed boot accelerometer must read -g");
    assert(sum(if plant.gear.contact[i] then 1 else 0 for i in 1:4)==4,"Expected four supporting feet");
  end when;
  when time>=2.5 then
    assert(abs(sum(plant.gear.normalForce))<1e-12 and truth.v_w[3]<0,"Composed vehicle must take off");
  end when;
end GroundRun;
''')
                files = [work/'generated/sources/fire_modelica_models/package.mo',work/'generated/GroundVehicle.mo',harness]
                script = work/'simulate.mos'
                script.write_text('loadModel(Modelica,{"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    +'simulate(GroundRun,stopTime=2.6,tolerance=1e-8,outputFormat="csv");\ngetErrorString();\n')
                run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                with (work/'GroundRun_res.csv').open() as handle:
                    last = list(csv.DictReader(handle))[-1]
                self.assertAlmostEqual(float(last['time']),2.6)
                self.assertLess(float(last['truth.p_w[3]']),0.2)

    def test_previous_namespace_outputs_can_be_regenerated(self):
        raw = 'schema_version=3\nmodel_name="MigratedQuad"'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work)
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            model = output / 'MigratedQuad.mo'
            model.write_text(model.read_text().replace('fire_modelica_models', 'FIRE_Modelica'))
            manifest_path = output / 'MigratedQuad.manifest.json'
            manifest = json.loads(manifest_path.read_text())
            manifest['generator_id'] = 'FIRE_Modelica/tools/fire-compose'
            manifest['generated_model_sha256'] = hashlib.sha256(model.read_bytes()).hexdigest()
            manifest['verification']['modelica_check'] = 'old-result'
            manifest_path.write_text(json.dumps(manifest))
            old = output / 'sources/FIRE_Modelica'
            (output / 'sources/fire_modelica_models').rename(old)
            (old / '.fire-compose').write_text('FIRE_Modelica/tools/fire-compose')
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertNotIn('FIRE_Modelica', model.read_text())
            self.assertIn('fire_modelica_models.Vehicles.', model.read_text())
            self.assertFalse(old.exists())
            self.assertTrue((output / 'sources/fire_modelica_models/package.mo').is_file())
            updated = json.loads(manifest_path.read_text())
            self.assertEqual(updated['generator_id'], 'fire_modelica_models/tools/fire-compose')
            self.assertEqual(updated['verification']['modelica_check'], 'not_run')

    def test_previous_namespace_cleanup_preserves_unowned_and_symlink_trees(self):
        raw = 'schema_version=3\nmodel_name="PreservedQuad"'
        for symlink in (False, True):
            with self.subTest(symlink=symlink), tempfile.TemporaryDirectory() as work:
                output = Path(work) / 'generated'
                sources = output / 'sources'
                sources.mkdir(parents=True)
                old = sources / 'FIRE_Modelica'
                if symlink:
                    target = Path(work) / 'external'
                    target.mkdir()
                    (target / '.fire-compose').write_text('FIRE_Modelica/tools/fire-compose')
                    old.symlink_to(target, target_is_directory=True)
                else:
                    old.mkdir()
                (old / 'user.mo').write_text('user-authored')
                run = self.run_config(raw, 'emit', '--output-dir', str(output))
                self.assertEqual(run.returncode, 0, run.stderr)
                self.assertEqual((old / 'user.mo').read_text(), 'user-authored')
                self.assertEqual(old.is_symlink(), symlink)

    def test_user_source_and_symlink_outputs_are_preserved(self):
        raw = 'schema_version=3\nmodel_name="KeepMe"'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work)
            source = output / 'KeepMe.mo'
            source.write_text('user-authored')
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(source.read_text(), 'user-authored')
            source.unlink()
            target = output / 'user.mo'
            target.write_text('user-authored')
            source.symlink_to(target)
            run = self.run_config(raw, 'emit', '--output-dir', work)
            self.assertNotEqual(run.returncode, 0)
            self.assertEqual(target.read_text(), 'user-authored')

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_quad_and_hexa_motor_response(self):
        for preset, count in [("QuadX",4),("HexaX",6)]:
            with self.subTest(preset=preset), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                raw = f'schema_version=3\nacquisition="continuous"\nmodel_name="ComposedVehicle"\n[geometry]\npreset="{preset}"\n'
                run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(work/'generated'))
                self.assertEqual(run.returncode, 0, run.stderr)
                harness = work/'MotorRun.mo'
                harness.write_text(f'model MotorRun\n  extends ComposedVehicle(pwm=fill(1500,{count}));\nend MotorRun;\n')
                script = work/'simulate.mos'
                files = [work/'generated/sources/fire_modelica_models/package.mo',work/'generated/ComposedVehicle.mo',harness]
                script.write_text('loadModel(Modelica, {"4.0.0"});\n'+''.join('loadFile('+json.dumps(str(f))+');\n' for f in files)
                    +'simulate(MotorRun, stopTime=0.1, tolerance=1e-9, numberOfIntervals=100, outputFormat="csv");\ngetErrorString();\n')
                run = subprocess.run(['omc',str(script)],cwd=work,text=True,capture_output=True,timeout=120)
                self.assertIn('The simulation finished successfully',run.stdout,run.stdout+run.stderr)
                with (work/'MotorRun_res.csv').open() as handle:
                    last = list(csv.DictReader(handle))[-1]
                self.assertAlmostEqual(float(last['time']),0.1)
                expected = 500*(1-math.exp(-0.1/0.03))
                for i in range(1,count+1):
                    self.assertAlmostEqual(float(last[f'rotorSpeed[{i}]']),expected,delta=1e-4)
                self.assertAlmostEqual(float(last['yaw_deg']),90.0,delta=1e-6)
                self.assertLess(float(last['accel[3]']),0)
                self.assertAlmostEqual(float(last['baro_altitude_m']),-float(last['truth.p_w[3]']),delta=1e-8)
                for key in ['gps[1]','gps[2]','gps[3]','accel[3]','baro_pressure_pa']:
                    self.assertTrue(math.isfinite(float(last[key])),key)
