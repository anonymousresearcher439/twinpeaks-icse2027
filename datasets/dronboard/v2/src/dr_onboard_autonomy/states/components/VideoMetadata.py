import atexit
import csv
import pathlib
import time

from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Dict

import rospy

DEFAULT_OUTPUT_DIR = Path.home() / "video-metadata"

_csv_file = None
_video_metadata_logger = None
_lock = RLock()

def get_output_path(t: time.struct_time=None):
    output_dir = rospy.get_param("~video_dir", DEFAULT_OUTPUT_DIR)
    if isinstance(output_dir, str):
        output_dir = Path(output_dir)

    if t == None:
        t = time.localtime()
    time_stamp = time.strftime("%Y-%m-%d_%H.%M.%S", t)
    return output_dir / f"{time_stamp}-video-metadata.csv"


def open_csv_writer():
    fields = [
        "timestamp",
        "drone latitude",
        "drone longitude",
        "drone altitude",

        "drone attitude i",
        "drone attitude j",
        "drone attitude k",
        "drone attitude w",

        "drone velocity E",
        "drone velocity N",
        "drone velocity U",

        "gimbal attitude i",
        "gimbal attitude j",
        "gimbal attitude k",
        "gimbal attitude w",

        "stare point latitude",
        "stare point longitude",
        "stare point altitude",
    ]

    global _video_metadata_logger
    global _lock

    with _lock:
        if _video_metadata_logger is not None:
            return
        out_path = get_output_path()
        rospy.loginfo(f"Writing Video Metadata to '{out_path}'")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        _csv_file = out_path.open(mode='w', newline='')
        atexit.register(_csv_file.close)
        _video_metadata_logger = csv.DictWriter(
            f=_csv_file,
            fieldnames=fields,
            dialect='excel',
            restval="",
            extrasaction='ignore'
        )
        _video_metadata_logger.writeheader()


def log_metadata(metadata: Dict):
    with _lock:
        if _video_metadata_logger is None:
            open_csv_writer()
        _video_metadata_logger.writerow(metadata)
