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
                "mqtt_local": {"host": "mqtt_local", "port": 1883},
                "gimbal": {
                    "pitch": True,
                    "roll": True,
                    "yaw": False,
                    "type": "gremsy",
                    "roll_limits": {"min": -60, "max": 60},
                    "pitch_limits": {"min": -127, "max": 127},
                    "axes": "roll_first"
                },
                "fcu": {"type": "ardupilot"},
                "camera": {
                    "horizontal_fov": 74.0,
                    "vertical_fov": 42.0,
                },
                "computer_vision": {
                    "use_bvhtree": False
                }
            }
        )

        self.assertEqual(expected, config)

    def test_load_file2(self):
        config_path = Path(__file__).parent / "config/test_configs/test_px4_config.toml"

        config = load_model(DRConfig, config_path, [])

        expected = DRConfig(
            **{
                "name": "Polkadot",
                "system_id": 1,
                "mqtt": {"host": "mqtt", "port": 1883},
                "mqtt_local": {"host": "mqtt_local", "port": 1883},
                "gimbal": {
                    "pitch": True,
                    "roll": True,
                    "yaw": True,
                    "type": "mavlink_gimbal_manager",
                    "roll_limits": {"min": -180, "max": 180},
                    "pitch_limits": {"min": -180, "max": 180},
                    "axes": "roll_first"
                },
                "fcu": {"type": "px4"}
            }
        )

        self.assertEqual(expected, config)

    def test_load_file4(self):
        config_path = Path(__file__).parent / "config/test_configs/test_ardu_mio.toml"

        config = load_model(DRConfig, config_path, [])

        expected = DRConfig(
            **{
                "name": "Polkadot",
                "system_id": 1,
                "mqtt": {"host": "mqtt", "port": 1883},
                "mqtt_local": {"host": "mqtt_local", "port": 1883},
                "gimbal": {
                    "pitch": True,
                    "roll": True,
                    "yaw": False,
                    "type": "gremsy",
                    "roll_limits": {"min": -60, "max": 60},
                    "pitch_limits": {"min": -127, "max": 127},
                    "axes": "roll_first"
                },
                "fcu": {"type": "ardupilot"},
                "camera": {
                    "horizontal_fov": 74.0,
                    "vertical_fov": 42.0
                }
            }
        )

        self.assertEqual(expected, config)

    def test_load_file_sade_data(self):
        config_path = Path(__file__).parent / "config/test_configs/sade-config.toml"

        config = load_model(DRConfig, config_path, [])
        drone_registration_id = "DRONE-POLKADOT-1000"
        pilot_id = "PILOT-POLKADOT-1000"
        owner_id = "OWNER-POLKADOT-1000"
        drone_model = "Skydio-2"
        self.assertEqual(config.drone_registration_id, drone_registration_id)
        self.assertEqual(config.pilot_id, pilot_id)
        self.assertEqual(config.owner_id, owner_id)
        self.assertEqual(config.drone_model, drone_model)

    def test_load_file_with_env_overrides(self):
        with EnvironmentVariableContext():
            config = load_model(DRConfig, self.config_path, [])

            expected = DRConfig(
                **{
                    "name": "Spot",  # Polkadot in file
                    "system_id": 1,
                    "mqtt": {"host": "mqtt", "port": 1883},
                    "mqtt_local": {"host": "mqtt_local", "port": 1883},
                    "gimbal": {
                        "pitch": False,  # True in file
                        "roll": True,
                        "yaw": False,
                        "type": "gremsy",
                        "roll_limits": {"min": -60, "max": 60},
                        "pitch_limits": {"min": -127, "max": 127},
                        "axes": "roll_first"
                    },
                    "fcu": {
                        "type": "ardupilot"
                    },
                    "camera": {
                        "horizontal_fov": 74.0,
                        "vertical_fov": 42.0,
                    },
                    "computer_vision": {
                        "use_bvhtree": False
                    }
                }
            )

            self.assertEqual(expected, config)

    def test_load_file_with_overrides(self):
        config = load_model(
            DRConfig,
            self.config_path,
            [(("gimbal", "type"), "mavlink_gimbal_manager")]
        )

        expected = DRConfig(
            **{
                "name": "Polkadot",
                "system_id": 1,
                "mqtt": {"host": "mqtt", "port": 1883},
                "mqtt_local": {"host": "mqtt_local", "port": 1883},
                "gimbal": {
                    "pitch": True,
                    "roll": True,
                    "yaw": False,
                    "type": "mavlink_gimbal_manager",  # gremsy in file
                    "roll_limits": {"min": -60, "max": 60},
                    "pitch_limits": {"min": -127, "max": 127},
                },
                "fcu": {
                    "type": "ardupilot"
                },
                "camera": {
                    "horizontal_fov": 74.0,
                    "vertical_fov": 42.0,
                },
                "computer_vision": {
                    "use_bvhtree": False
                }
            }
        )
        print(config)
        self.assertEqual(expected, config)

    def test_load_file_with_env_overrides_and_overrides(self):
        with EnvironmentVariableContext():
            config = load_model(
                DRConfig, self.config_path, [(("gimbal", "type"), "mavlink_gimbal_manager")]
            )

            expected = DRConfig(
                **{
                    "name": "Spot",  # Polkadot in file
                    "system_id": 1,
                    "mqtt": {"host": "mqtt", "port": 1883},
                    "mqtt_local": {"host": "mqtt_local", "port": 1883},
                    "gimbal": {
                        "pitch": False,  # True in file
                        "roll": True,
                        "yaw": False,
                        "type": "mavlink_gimbal_manager",  # gremsy in file
                        "roll_limits": {"min": -60, "max": 60},
                        "pitch_limits": {"min": -127, "max": 127},
                    },
                    "fcu": {
                        "type": "ardupilot"
                    },
                    "camera": {
                        "horizontal_fov": 74.0,
                        "vertical_fov": 42.0,
                    },
                    "computer_vision": {
                        "use_bvhtree": False
                    }
                }
            )

            self.assertEqual(expected, config)
