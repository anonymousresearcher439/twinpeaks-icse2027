import argparse
import logging
from logging.handlers import RotatingFileHandler
import re
import subprocess
from time import sleep
from typing import Dict, List, Optional, Tuple


log = logging.getLogger("ttl_device_reset")
log_format = logging.Formatter(
    fmt="%(asctime)s %(levelname)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

log_file_handler = RotatingFileHandler(
    filename="/var/log/dr-ttlreset.log",
    maxBytes=1E7, #10 MB
    backupCount=1
)
log_file_handler.setFormatter(log_format)

log_stream_handler = logging.StreamHandler()
log_stream_handler.setFormatter(log_format)

log.addHandler(log_file_handler)
log.addHandler(log_stream_handler)

log.setLevel(logging.INFO)


def check_docker_services(services_to_check: Tuple[str]) -> Optional[Tuple[str]]:
    """checks the running docker services and returns the names of any services that aren't
    running, but should be

    Parameters
    ----------
    services_to_check:
        names of the docker services to check

    Returns
    -------
    failed services or
    None if both services are running
    """
    result = subprocess.run(["docker", "ps"], stdout=subprocess.PIPE, text=True)

    delimited_rows = []
    for row in re.split("\n", result.stdout):
        delimited_rows.append(re.split(r"\s{2,}", row))

    running_services: List[str] = []
    for i, row in enumerate(row for row in delimited_rows if len(row) >= 6):
        if i > 0:
            running_services.append(row[-1])

    missing_services = list(services_to_check)
    for running_service in running_services:
        for service_to_check in missing_services:
            if running_service.find(service_to_check) > -1:
                missing_services.remove(service_to_check)

    return tuple(missing_services)


def find_usb_devices_to_reset(
    missing_services: Tuple[str],
    service_to_device_map: Dict[str, str]
) -> Tuple[str]:
    """look up the bus/device number of the usb to ttl serial devices associated with the provided
    missing services

    Returns
    -------
    tuple of busnum/devnum representing all usb devices that need reset
    """
    devices_to_reset = []

    for service in missing_services:
        udev_process = subprocess.Popen(
            ["udevadm", "info", f"--name={service_to_device_map[service]}", "--attribute-walk"],
            stdout=subprocess.PIPE, text=True
        )
        grep_process = subprocess.Popen(
            ["grep", "-E", "-m 2", "busnum|devnum"],
            stdin=udev_process.stdout, stdout=subprocess.PIPE, text=True
        )
        output, error = grep_process.communicate()
        
        for row in re.split("\n", output):
            if "busnum" in row:
                busnum = row.split("\"")[1].zfill(3)
            if "devnum" in row:
                devnum = row.split("\"")[1].zfill(3)
        
        log.info(f"{service} uses usb device {busnum}/{devnum} (busnum/devnum)")
        devices_to_reset.append(f"{busnum}/{devnum}")

    return tuple(devices_to_reset)


def reset_usb_device(usb_device: str) -> bool:
    """resets a usb device given a string representing its bus and device number in the format
    'BBB/DDD'

    Returns
    -------
    True if reset was successful
    """
    result = subprocess.run(["usbreset", usb_device], stdout=subprocess.PIPE, text=True)
    
    if "ok" in result.stdout:
        return True
    
    return False


def start_docker_service(path_to_docker_compose: str, service_to_start: str) -> bool:
    """starts a given docker compose service in the docker compose file at the path provided

    Returns
    -------
    True if the service is started successfully, False otherwise
    """
    # use -d option with docker compose up to run container in background
    result = subprocess.run(
        ["docker", "compose", "-f", path_to_docker_compose, "up", "-d", service_to_start],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # if have a warning from docker (ie. orphaned containers), then full start log may be published
    # to stderr
    if ("Started" in result.stderr
    or "Started" in result.stderr):
        return True
    
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "docker_compose_path",
        help="absolute path to docker compose file containing services to check"
    )
    parser.add_argument(
        "services_to_check",
        nargs="+",
        help="docker compose services that will be checked and restarted if necessary"
    )
    parser.add_argument(
        "-p",
        "--period",
        type=float,
        default=10.0,
        help="time in seconds between service checks"
    )

    args = parser.parse_args()
    docker_compose_path: str = args.docker_compose_path
    services_to_check: List[str] = args.services_to_check
    time_between_checks: float = args.period

    log.info("--- ttl_device_reset starting ---")
    log.info("checking services:")
    for service in services_to_check:
        log.info(f"- {service}")

    while(True):
        sleep(time_between_checks)

        missing_services = check_docker_services(tuple(services_to_check))
        if not missing_services:
            log.info("all services are running")
            continue
        
        for service in missing_services:
            log.warning(f"{service} failed")

        service_to_device_map = {
            "mavros" : "/dev/ttyMAVROS",
            "mavlink_router_gcs" : "/dev/ttyGCS"
        }

        usb_devices_by_bus_device_name = find_usb_devices_to_reset(
            missing_services,
            service_to_device_map
        )
        log.debug(f"usb devices to reset - {usb_devices_by_bus_device_name}")

        for usb_device in usb_devices_by_bus_device_name:
            if reset_usb_device(usb_device):
                log.info(f"{usb_device} reset successfully")
            else:
                log.warning(f"{usb_device} failed to reset")


        for service in missing_services:
            if start_docker_service(docker_compose_path, service):
                log.info(f"started docker service: {service}")
            else:
                log.warning(f"failed to start docker service: {service}")


if __name__ == "__main__":
    main()

