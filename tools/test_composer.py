"""Rust frontend parity and artifact behavior: python3 -m unittest discover -s tools -p test_composer.py."""
import json
import hashlib
import math
import os
import csv
from pathlib import Path
import subprocess
import tempfile
import unittest

import generate_config as reference
try:
    import tomllib as tomli
except ModuleNotFoundError:
    import tomli

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

    def test_schema_one_example_parity(self):
        for source in sorted((ROOT / 'configs').glob('*.toml')):
            with self.subTest(source=source.name):
                raw = source.read_text()
                expected = reference.validate(tomli.loads(raw))
                run = self.run_config(raw, 'validate', '--json')
                self.assertEqual(run.returncode, 0, run.stderr)
                actual = json.loads(run.stdout)
                actual.pop('acquisition')
                self.assert_config_equal(expected, actual)

    def test_nested_physics_table_matches_standalone_configuration(self):
        standalone = ('schema_version=2\nmodel_name="NestedHexa"\nacquisition="continuous"\n'
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
        valid = '[physics]\nschema_version=2\nmodel_name="SelectedQuad"\n'
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
            'schema_version=2\nacquisition=false',
            'schema_version=2\nacquisition="unknown"',
            'schema_version=1\nmodel_name="Bad; end Bad"',
            'schema_version=1\n[motor]\ntau=nan',
            'schema_version=1\n[motor]\ntau=true',
            'schema_version=1\n[geometry]\nactuatorIndex=[1,2,3,5]',
            'schema_version=1\n[mass.aggregate]\ninertia=[[1,0,0],[0,1,0],[0,0,3]]',
            'schema_version=2\n[models.imu]\nresponse="first_order"\ntau=[0.1,0.1]',
        ]
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            for raw in cases:
                with self.subTest(raw=raw):
                    run = self.run_config(raw, 'emit', '--output-dir', str(output))
                    self.assertNotEqual(run.returncode, 0)
                    self.assertFalse(output.exists())

    def test_artifacts_contain_one_plant_and_track_acquisition(self):
        raw = 'schema_version=2\nmodel_name="TestHexa"\nacquisition="continuous"\n[geometry]\npreset="HexaX"\nnActuators=8\nactuatorIndex=[1,2,3,4,6,8]\n'
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / 'generated'
            run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(output))
            self.assertEqual(run.returncode, 0, run.stderr)
            source = (output / 'TestHexa.mo').read_text()
            self.assertEqual(source.count('MultirotorWithSensors plant('), 1)
            self.assertIn('sampledSensors=false', source)
            self.assertIn('sampledActuators=false', source)
            manifest = json.loads((output / 'TestHexa.manifest.json').read_text())
            self.assertEqual(manifest['interface']['nActuators'], 8)
            self.assertFalse(manifest['event_requirements']['sampled_sensors'])
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

    def test_previous_namespace_outputs_can_be_regenerated(self):
        raw = 'schema_version=1\nmodel_name="MigratedQuad"'
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
        raw = 'schema_version=1\nmodel_name="PreservedQuad"'
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
        raw = 'schema_version=1\nmodel_name="KeepMe"'
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
    def test_continuous_and_sampled_acquisition_simulate(self):
        with tempfile.TemporaryDirectory() as work:
            work = Path(work)
            raw = 'schema_version=2\nacquisition="continuous"\nmodel_name="CheckQuad"'
            run = self.run_config(raw, 'emit', '--profile', 'fastdyn', '--output-dir', str(work/'generated'))
            self.assertEqual(run.returncode, 0, run.stderr)
            package = work/'generated/sources/fire_modelica_models/package.mo'
            script = work/'check.mos'
            script.write_text('loadModel(Modelica, {"4.0.0"});\nloadFile('+json.dumps(str(package))+');\n'
                'simulate(fire_modelica_models.Tests.AcquisitionProfiles, stopTime=0.035, numberOfIntervals=70, outputFormat="csv");\ngetErrorString();\n')
            run = subprocess.run(['omc', str(script)], cwd=work, text=True, capture_output=True, timeout=120)
            self.assertEqual(run.returncode, 0, run.stdout+run.stderr)
            result = work/'fire_modelica_models.Tests.AcquisitionProfiles_res.csv'
            self.assertTrue(result.is_file(), run.stdout+run.stderr)
            self.assertIn('The simulation finished successfully', run.stdout)
            with result.open() as handle:
                rows = list(csv.DictReader(handle))
            last = rows[-1]
            self.assertAlmostEqual(float(last['time']), 0.035)
            self.assertAlmostEqual(float(last['sensors[1].measurements.position[1]']), 0.035)
            self.assertAlmostEqual(float(last['sensors[2].measurements.position[1]']), 0.03)
            self.assertAlmostEqual(float(last['commands[1].demand[6]']), 0.35)
            self.assertAlmostEqual(float(last['commands[2].demand[6]']), 0.3)

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "set FIRE_TEST_OMC=1")
    def test_composed_quad_and_hexa_motor_response(self):
        for preset, count in [("QuadX",4),("HexaX",6)]:
            with self.subTest(preset=preset), tempfile.TemporaryDirectory() as work:
                work = Path(work)
                raw = f'schema_version=2\nacquisition="continuous"\nmodel_name="ComposedVehicle"\n[geometry]\npreset="{preset}"\n'
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
                self.assertAlmostEqual(float(last['values.measurements.imuSampleTime']),0.1)
                for key in ['gps[1]','gps[2]','gps[3]','accel[3]','baro_pressure_pa']:
                    self.assertTrue(math.isfinite(float(last[key])),key)
