import unittest
import os
from pathlib import Path

from dr_onboard_autonomy.common.loaders import load_env_overrides, load_model
from dr_onboard_autonomy.models import DRConfig


class EnvironmentVariableContext:
    def __enter__(self):
        os.environ["DR__NAME"] = "Spot"
        os.environ["DR__GIMBAL__PITCH"] = "false"

    def __exit__(self, exc_type, exc_value, traceback):
        del os.environ["DR__NAME"]
        del os.environ["DR__GIMBAL__PITCH"]


class TestLoaderDRConfig(unittest.TestCase):
    @classmethod
    def setUp(cls) -> None:
        cls.config_path = Path(__file__).parent / "config/config.toml"

    def test_load_env_overrides(self):
        with EnvironmentVariableContext():
            # NOTE: bools won't be coerced until they are parsed by the model
            expected = {
                "name": "Spot",
                "gimbal": {"pitch": "false"},
            }

            self.assertEqual(expected, load_env_overrides({}))

    def test_load_file(self):
        config = load_model(DRConfig, self.config_path, [])

        expected = DRConfig(
            **{
                "name": "Polkadot",
                "system_id": 1,
                "mqtt": {"host": "mqtt", "port": 1883},
                "gimbal": {"pitch": True, "roll": True, "yaw": False, "type": "gremsy"},
            }
        )

        self.assertEqual(expected, config)

    def test_load_file_with_env_overrides(self):
        with EnvironmentVariableContext():
            config = load_model(DRConfig, self.config_path, [])

            expected = DRConfig(
                **{
                    "name": "Spot",  # Polkadot in file
                    "system_id": 1,
                    "mqtt": {"host": "mqtt", "port": 1883},
                    "gimbal": {
                        "pitch": False,  # True in file
                        "roll": True,
                        "yaw": False,
                        "type": "gremsy",
                    },
                }
            )

            self.assertEqual(expected, config)

    def test_load_file_with_overrides(self):
        config = load_model(DRConfig, self.config_path, [(("gimbal", "type"), "px4")])

        expected = DRConfig(
            **{
                "name": "Polkadot",
                "system_id": 1,
                "mqtt": {"host": "mqtt", "port": 1883},
                "gimbal": {
                    "pitch": True,
                    "roll": True,
                    "yaw": False,
                    "type": "px4",  # gremsy in file
                },
            }
        )

        self.assertEqual(expected, config)

    def test_load_file_with_env_overrides_and_overrides(self):
        with EnvironmentVariableContext():
            config = load_model(
                DRConfig, self.config_path, [(("gimbal", "type"), "px4")]
            )

            expected = DRConfig(
                **{
                    "name": "Spot",  # Polkadot in file
                    "system_id": 1,
                    "mqtt": {"host": "mqtt", "port": 1883},
                    "gimbal": {
                        "pitch": False,  # True in file
                        "roll": True,
                        "yaw": False,
                        "type": "px4",  # gremsy in file
                    },
                }
            )

            self.assertEqual(expected, config)
