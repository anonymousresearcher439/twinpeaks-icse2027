#!/usr/bin/env python3

import unittest
import datetime

from pathlib import Path
from unittest.mock import patch

import dr_onboard_autonomy.states.components.VideoMetadata as VideoMetadata
from dr_onboard_autonomy.states.components.VideoMetadata import (
    get_output_path,
    log_metadata,
)

def mock_rospy_get_param_default(param_name, default_value):
    return default_value


def mock_rospy_get_param_non_default(param_name, default_value):
    return "/mnt/video/video-metadata"

class TestVideoMetadata(unittest.TestCase):
    @patch(
        target="dr_onboard_autonomy.states.components.VideoMetadata.DEFAULT_OUTPUT_DIR",
        new=Path("/home/example/video-metadata"),
    )
    @patch(target="rospy.get_param", new=mock_rospy_get_param_default)
    def test_get_output_path(self):
        day = datetime.date(2022, 3, 30)
        hour = datetime.time(hour=12, minute=11, second=10)
        t = datetime.datetime.combine(day, hour)
        t = t.timetuple()

        actual_path = get_output_path(t)
        expected_path = Path("/home/example/video-metadata/2022-03-30_12.11.10-video-metadata.csv")

        self.assertEqual(actual_path, expected_path)

    @patch(
        target="dr_onboard_autonomy.states.components.VideoMetadata.DEFAULT_OUTPUT_DIR",
        new=Path("/home/example/video-metadata"),
    )
    @patch(target="rospy.get_param", new=mock_rospy_get_param_non_default)
    def test_get_output_path_with_rospy_param_providing_the_output_dir(self):
        day = datetime.date(2022, 3, 30)
        hour = datetime.time(hour=12, minute=11, second=10)
        t = datetime.datetime.combine(day, hour)
        t = t.timetuple()
        

        actual_path = get_output_path(t)

        file_name = "2022-03-30_12.11.10-video-metadata.csv"
        dir_name = mock_rospy_get_param_non_default(None, None)
        expected_path = Path(dir_name) / file_name

        self.assertEqual(actual_path, expected_path)

    @patch(
        target="dr_onboard_autonomy.states.components.VideoMetadata.DEFAULT_OUTPUT_DIR",
        new=Path("/home/example/video-metadata"),
    )
    @patch(target="rospy.get_param", new=mock_rospy_get_param_default)
    def test_get_output_path_no_time_struct(self):
        actual_path = get_output_path()
        actual_str = str(actual_path)
        
        self.assertTrue("/home/example/video-metadata" in actual_str)
        self.assertTrue("-video-metadata.csv" in actual_str)

        # hopefully the year doesn't change while we're running this test
        dt = datetime.datetime.now()
        year_str = f"{dt.year}"
        self.assertTrue(year_str in actual_str)

    @patch(
        target="dr_onboard_autonomy.states.components.VideoMetadata.DEFAULT_OUTPUT_DIR",
        new=Path("/tmp/video-metadata"),
    )
    @patch(target="rospy.get_param", new=mock_rospy_get_param_default)
    def test_log_metadata(self):
        data = {
            "timestamp": 0.0,
            "drone latitude": 0.0,
            "drone longitude": 0.0,
            "drone altitude": 0.0,
            "drone attitude i": 0.0,
            "drone attitude j": 0.0,
            "drone attitude k": 0.0,
            "drone attitude w": 0.0,
            "drone velocity E": 0.0,
            "drone velocity N": 0.0,
            "drone velocity U": 0.0,
            "gimbal attitude i": 0.0,
            "gimbal attitude j": 0.0,
            "gimbal attitude k": 0.0,
            "gimbal attitude w": 0.0,
            "stare point latitude": 0.0,
            "stare point longitude": 0.0,
            "stare point altitude": 0.0,
        }

        log_metadata(data)

    
    
        
