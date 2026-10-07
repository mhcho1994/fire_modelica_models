"""Run: python3 -m unittest discover -s tools -p test_generate_config.py

Set FIRE_TEST_OMC=1 to additionally check every example with OpenModelica.
"""
import copy
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import generate_config as generator


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.config = {"schema_version": 1, "model_name": "UnitQuad", "geometry": {"preset": "QuadX"}}

    def rejects(self, config, phrase):
        with self.assertRaisesRegex(generator.ConfigError, phrase):
            generator.validate(config)

    def test_unknown_fields_and_schema_are_rejected(self):
        self.rejects(dict(self.config, electrical={}), "unknown fields")
        self.rejects(dict(self.config, schema_version=True), "schema_version")
        self.config["motor"] = {"tao": 0.03}
        self.rejects(self.config, "unknown fields")

    def test_identifier_cannot_inject_modelica(self):
        self.rejects(dict(self.config, model_name="Quad; end Evil"), "identifier")

    def test_dimension_and_channel_bounds(self):
        self.config["geometry"]["actuatorIndex"] = [1, 2, 3]
        self.rejects(self.config, "dimensions")
        self.config["geometry"]["actuatorIndex"] = [1, 2, 3, 5]
        self.rejects(self.config, "exceeds")
        self.config["geometry"]["actuatorIndex"] = [1, 2, 3, 4.0]
        self.rejects(self.config, "integer")

    def test_valid_sparse_command_mapping(self):
        self.config["geometry"].update(nActuators=8, actuatorIndex=[1, 3, 5, 8])
        self.assertEqual(generator.validate(self.config)["geometry"]["actuatorIndex"], [1, 3, 5, 8])

    def test_custom_geometry_and_per_rotor_parameters(self):
        mounts = [[.2, 0., .03], [0., .3, 0.], [-.4, 0., 0.], [0., -.25, 0.]]
        rotated = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
        self.config["geometry"].update(rotorPosition_C=mounts,
            R_br=[rotated for _ in range(4)], spinSign=[-1, 1, -1, 1])
        self.config["motor"] = {"tau": [.01, .02, .03, .04]}
        resolved = generator.validate(self.config)
        self.assertEqual(resolved["geometry"]["rotorPosition_C"], mounts)
        self.assertEqual(resolved["geometry"]["R_br"][0], rotated)
        self.assertEqual(resolved["motor"]["tau"], [.01, .02, .03, .04])

    def test_custom_rotation_rejects_reflection_and_nonorthogonality(self):
        rotations = [copy.deepcopy(generator.IDENTITY) for _ in range(4)]
        rotations[0][0][0] = -1
        self.config["geometry"]["R_br"] = rotations
        self.rejects(self.config, "determinant")
        rotations[0][0][0] = 2
        self.rejects(self.config, "orthonormal")

    def test_invalid_inertia_and_physical_values(self):
        self.config["mass"] = {"aggregate": {"mass": 1, "inertia": [[1, 2, 0], [2, 1, 0], [0, 0, 1]]}}
        self.rejects(self.config, "positive definite")
        del self.config["mass"]
        for value in [float("nan"), float("inf"), -0.1, 0, True]:
            self.config["motor"] = {"tau": value}
            self.rejects(self.config, "motor.tau")
        self.config["motor"] = {"omega_start": 1001}
        self.rejects(self.config, "omega_start")

    def test_mass_modes_do_not_silently_double_count(self):
        self.config["mass"] = {"mode": "aggregate", "payloads": []}
        self.rejects(self.config, "constituent")
        self.config["mass"] = {"mode": "assembled"}
        self.rejects(self.config, "requires core and arm")

    def test_positive_definite_but_nonphysical_inertia_is_rejected(self):
        self.config["mass"] = {"aggregate": {"mass": 1,
            "inertia": [[1, 0, 0], [0, 1, 0], [0, 0, 3]]}}
        self.rejects(self.config, "physical mass distribution")
        # Rotating the impossible principal moments must not bypass validation.
        self.config["mass"]["aggregate"]["inertia"] = [[2, 1, 0], [1, 2, 0], [0, 0, 1]]
        self.rejects(self.config, "physical mass distribution")
        self.config["mass"]["aggregate"]["inertia"] = [[1, 0, 0], [0, 1, 0], [0, 0, 2]]
        generator.validate(self.config)  # Planar mass at the triangle-equality boundary is allowed.

    def test_component_ids_are_unique_in_selected_mass_budget(self):
        self.config["mass"] = {"mode": "assembled",
            "core": {"mass": 1, "inertia": generator.IDENTITY, "componentId": "frame.core"},
            "arm": {"mass": .1, "inertia": generator.IDENTITY},
            "payloads": [{"mass": .1, "inertia": generator.IDENTITY, "componentId": "camera"}]}
        resolved = generator.validate(self.config)
        self.assertEqual([p["componentId"] for p in resolved["mass"]["arms"]],
                         ["arm.1", "arm.2", "arm.3", "arm.4"])
        self.assertIn('componentId="camera"', generator.render(resolved))
        self.config["mass"]["payloads"][0]["componentId"] = "frame.core"
        self.rejects(self.config, "duplicate")
        self.config["mass"]["payloads"][0]["componentId"] = "arm.1"
        self.rejects(self.config, "duplicate")
        self.config["mass"]["payloads"][0]["componentId"] = ""
        self.config["mass"]["arm"]["componentIds"] = ["a", "b", "c", "a"]
        self.rejects(self.config, "duplicate")
        self.config["mass"]["arm"]["componentIds"] = ["a", "b"]
        self.rejects(self.config, "expected 4")

    def test_component_id_does_not_accept_modelica_text(self):
        self.config["mass"] = {"aggregate": {"mass": 1, "inertia": generator.IDENTITY,
                                              "componentId": 'camera"; end Bad'}}
        self.rejects(self.config, "componentId")

    def test_presets_and_zero_legs(self):
        for preset, (_, nr, _) in generator.PRESETS.items():
            self.config["geometry"] = {"preset": preset, "nLegs": 0}
            resolved = generator.validate(self.config)
            self.assertEqual(resolved["geometry"]["nRotors"], nr)
            self.assertEqual(resolved["geometry"]["legPosition_C"], [])
            text = generator.render(resolved)
            self.assertNotIn("legPosition_C={}", text)
            self.assertNotIn("demand=", text)
        geometry = resolved["geometry"]
        self.assertEqual(geometry["spinSign"], [1, -1, 1, -1, -1, 1, -1, 1])
        self.assertEqual([p[2] for p in geometry["rotorPosition_C"]], [.025] * 4 + [-.025] * 4)

    def test_invalid_configuration_writes_nothing(self):
        with tempfile.TemporaryDirectory() as work:
            config = Path(work) / "invalid.toml"
            config.write_text('schema_version=1\n[geometry]\nactuatorIndex=[1,2,3,7]\n')
            output = Path(work) / "generated"
            with self.assertRaises(generator.ConfigError):
                generator.generate(config, output)
            self.assertFalse(output.exists())

    def test_source_destination_is_rejected_without_touching_existing_source(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work) / "source"
            source = root / "Vehicles" / "Copter"
            source.mkdir(parents=True)
            original = source / "MultirotorPlant.mo"
            original.write_text("within Vehicles.Copter; model MultirotorPlant end MultirotorPlant;\n")
            before = original.read_bytes()
            config = Path(work) / "config.toml"
            config.write_text('schema_version=1\nmodel_name="MultirotorPlant"\n')
            with patch.object(generator, "ROOT", root):
                for output in (source, root / "new_source_folder"):
                    with self.assertRaisesRegex(generator.ConfigError, "outside the source root"):
                        generator.generate(config, output)
            self.assertEqual(original.read_bytes(), before)
            self.assertFalse((root / "new_source_folder").exists())
            self.assertFalse((source / "MultirotorPlant.manifest.json").exists())

    def test_unowned_existing_model_and_manifest_are_not_overwritten(self):
        with tempfile.TemporaryDirectory() as work:
            output = Path(work) / "output"
            output.mkdir()
            config = Path(work) / "config.toml"
            config.write_text('schema_version=1\nmodel_name="UnitQuad"\n')
            model = output / "UnitQuad.mo"
            model.write_text("model UnitQuad end UnitQuad;\n")
            with self.assertRaisesRegex(generator.ConfigError, "non-generated Modelica"):
                generator.generate(config, output)
            self.assertEqual(model.read_text(), "model UnitQuad end UnitQuad;\n")
            # A separately created user manifest must also stop the entire write.
            model.write_text(generator.render(generator.validate(self.config)))
            before = model.read_bytes()
            manifest = output / "UnitQuad.manifest.json"
            manifest.write_text('{"user_data": true}\n')
            with self.assertRaisesRegex(generator.ConfigError, "non-generated manifest"):
                generator.generate(config, output)
            self.assertEqual(model.read_bytes(), before)
            self.assertEqual(manifest.read_text(), '{"user_data": true}\n')

    def test_symlink_output_cannot_redirect_into_source(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work) / "source"
            source = root / "Vehicles"
            source.mkdir(parents=True)
            build = root / "build"
            build.mkdir()
            try:
                (build / "escape").symlink_to(source, target_is_directory=True)
            except (OSError, NotImplementedError):
                self.skipTest("Directory symlinks unavailable")
            config = Path(work) / "config.toml"
            config.write_text('schema_version=1\nmodel_name="UnitQuad"\n')
            with patch.object(generator, "ROOT", root):
                with self.assertRaisesRegex(generator.ConfigError, "outside the source root"):
                    generator.generate(config, build / "escape")
            self.assertFalse((source / "UnitQuad.mo").exists())
            original = source / "User.mo"
            original.write_text("model User end User;\n")
            (build / "UnitQuad.mo").symlink_to(original)
            with patch.object(generator, "ROOT", root):
                with self.assertRaisesRegex(generator.ConfigError, "symlink output"):
                    generator.generate(config, build)
            self.assertEqual(original.read_text(), "model User end User;\n")

    def test_previous_namespace_outputs_can_be_regenerated(self):
        with tempfile.TemporaryDirectory() as work:
            config = generator.ROOT / "configs/quad.toml"
            model, manifest_path = generator.generate(config, work)
            model.write_text(model.read_text().replace("fire_modelica_models", "FIRE_Modelica"))
            manifest = json.loads(manifest_path.read_text())
            manifest["generator_id"] = "FIRE_Modelica/tools/generate_config.py"
            manifest["generated_model_sha256"] = generator.hashlib.sha256(model.read_bytes()).hexdigest()
            manifest_path.write_text(json.dumps(manifest))
            generator.generate(config, work)
            self.assertIn("extends fire_modelica_models.", model.read_text())
            self.assertNotIn("FIRE_Modelica", model.read_text())
            updated = json.loads(manifest_path.read_text())
            self.assertEqual(updated["generator_id"], generator.GENERATOR_ID)
            self.assertEqual(updated["generated_model_sha256"], generator.hashlib.sha256(model.read_bytes()).hexdigest())

    def test_repeated_generation_has_stable_source_digest(self):
        with tempfile.TemporaryDirectory() as work:
            root = Path(work) / "source"
            root.mkdir()
            (root / "package.mo").write_text("package Source end Source;\n")
            config = Path(work) / "config.toml"
            config.write_text('schema_version=1\nmodel_name="UnitQuad"\n')
            with patch.object(generator, "ROOT", root):
                for output in (root / "build" / "custom", Path(work) / "external"):
                    _, first_path = generator.generate(config, output)
                    first = json.loads(first_path.read_text())
                    _, second_path = generator.generate(config, output)
                    second = json.loads(second_path.read_text())
                    self.assertEqual(first["source"]["sha256"], second["source"]["sha256"])
                    self.assertEqual(first["source"]["files"], {"package.mo": first["source"]["files"]["package.mo"]})
                    self.assertEqual(first["generated_model_sha256"], second["generated_model_sha256"])

    def test_example_generation_and_manifest(self):
        with tempfile.TemporaryDirectory() as work:
            for path in sorted((generator.ROOT / "configs").glob("*.toml")):
                model, manifest_path = generator.generate(path, work)
                manifest = json.loads(manifest_path.read_text())
                self.assertIn("extends fire_modelica_models.Vehicles.Copter.MultirotorWithSensors", model.read_text())
                self.assertEqual(manifest["target"], "openmodelica-native")
                self.assertEqual(manifest["verification"]["simulation"], "not_run")
                self.assertEqual(manifest["verification"]["openmodelica_checkModel"], "not_run")
                self.assertEqual(len(manifest["source"]["sha256"]), 64)
                self.assertTrue(manifest["event_requirements"]["sampled_sensors"])
                self.assertIn("EMI", manifest["excluded_capabilities"])

    @unittest.skipUnless(os.environ.get("FIRE_TEST_OMC") == "1", "Set FIRE_TEST_OMC=1 for compiler checks")
    def test_examples_with_openmodelica(self):
        with tempfile.TemporaryDirectory() as work:
            for path in sorted((generator.ROOT / "configs").glob("*.toml")):
                with self.subTest(config=path.name):
                    _, manifest_path = generator.generate(path, work, check=True)
                    manifest = json.loads(manifest_path.read_text())
                    self.assertEqual(manifest["verification"]["openmodelica_checkModel"], "passed")
            for preset in generator.PRESETS:
                with self.subTest(preset=preset, nLegs=0):
                    config = generator.validate({"schema_version": 1, "model_name": "NoLeg" + preset,
                                                 "geometry": {"preset": preset, "nLegs": 0}})
                    model = Path(work) / (config["model_name"] + ".mo")
                    model.write_text(generator.render(config))
                    generator.check_model(shutil.which("omc"), model, config["model_name"])


if __name__ == "__main__":
    unittest.main()
