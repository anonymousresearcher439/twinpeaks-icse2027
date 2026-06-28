#!/bin/bash

# This script will setup a NTP server on a laptop and configure the Jetson to
# sync its clock with the laptop.

# RUN THIS SCRIPT AS ROOT
# print an error if not root
if [ "$(id -u)" != "0" ]; then
   echo "This script must be run as root" 1>&2
   exit 1
fi

# if not interactive shell, set a default timezone to Indianapolis
if [ -z "$PS1" ]; then
    # set a default timezone
    export DEBIAN_FRONTEND=noninteractive
fi

# setup default values

# Set default value for laptop host/IP address
laptop_host="10.223.0.1"

# Set default values based on system architecture
if [ "$(uname -m)" == "aarch64" ]; then
    setup_jetson=true
    setup_laptop=false
else
    setup_laptop=true
    setup_jetson=false
fi

# Parse arguments
while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        --setup-laptop*)
        setup_laptop=true
        if [[ "$arg" == *=* ]]; then
            setup_laptop="${arg#*=}"
        fi
        shift
        ;;
        --setup-jetson*)
        setup_jetson=true
        if [[ "$arg" == *=* ]]; then
            setup_jetson="${arg#*=}"
        fi
        shift
        ;;
        --laptop-host=*)
        laptop_host="${arg#*=}"
        shift
        ;;
        -h|--help)
        echo "Usage: $0 [OPTIONS]"
        echo ""
        echo "OPTIONS:"
        echo "--setup-laptop[=VALUE]: Setup NTP server. VALUE defaults to $setup_laptop."
        echo "--setup-jetson[=VALUE]: Configure system clock to sync with laptop. VALUE defaults to $setup_jetson."
        echo "--laptop-host=HOST: Set the laptop host/IP address (default: 10.223.0.1)."
        echo "-h, --help: Show help and exit."
        exit 0
        ;;
        *)
        echo "Unknown argument: $arg"
        exit 1
        ;;
    esac
done

setup_ntp_server () {
    # install ntp
    apt-get update
    apt-get install --yes ntp
}

setup_jetson_clock () {
    # install systemd-timesyncd
    apt-get update
    apt-get install --yes systemd-timesyncd
    # set the time server IP address to 10.223.0.1
    # Edit the /etc/systemd/timesyncd.conf file and set the NTP server
    echo "Setting NTP server to $1..."
    echo "NTP=$1" >> /etc/systemd/timesyncd.conf
    # configure systemd-timesyncd
    systemctl restart systemd-timesyncd
    systemctl enable systemd-timesyncd
}

# Do setup based on arguments
if [ "$setup_laptop" = true ]; then
    echo "Setting up NTP server..."
    setup_ntp_server
fi

if [ "$setup_jetson" = true ]; then
    echo "Configuring system clock to sync with laptop..."
    setup_jetson_clock $laptop_host
fi


