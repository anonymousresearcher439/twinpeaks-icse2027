#!/usr/bin/env python3
# This script will start gazebo And wait for it to download model files.
# Why?
# When gazebo is started for the first time, it will download models from the internet.
# This process can take a while depending on the internet speed.
# We will have trouble starting ardupilot SITL if gazebo is still downloading models.
# ardupilot SITL will time out when it tries to connect to gazebo.
# and the simulation will not start.
#
# Context:
# This documentation explains how to setup ardupilot SITL with gazebo.
# https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html#sitl-with-gazebo-legacy
# 
# One of the instructions is as follows:
#
# > The first time gazebo is executed requires the download of some models and it could take some time, so please be patient. Let’s give a try.
# ```bash
# gazebo --verbose
# ```
#
# This script automates the process of running gazebo and waiting for it to download models.
# it watches the stdout and stderr of the gazebo process. When it sees that gazebo is downloading models,
# it will exit.
import re
import select
import subprocess
import sys
import time


MAX_WAIT_TIME = 600  # 10 minutes

# Regular expression to remove ANSI escape codes
# gazebo uses ANSI escape codes to colorize the output.
# this regex will remove the color codes.
# this lets us compare the output of gazebo with a string.
ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')


def run_gzserver():
    cmd = ['gzserver', '--verbose', '/usr/local/ardupilot_gazebo/worlds/iris_arducopter_runway.world']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    world_loaded = False
    models_downloading = False

    start = time.time()
    try:
        # Monitor both stdout and stderr without blocking
        while True:
            if time.time() - start > MAX_WAIT_TIME:
                print("ERROR! Max wait time exceeded. Terminating gzserver.", file=sys.stderr)
                exit(1)
    
            # Wait until there is something to read
            readable, _, _ = select.select([process.stdout, process.stderr], [], [], 5)

            for stream in readable:
                line = stream.readline()
                line = str(line)
                line = ansi_escape.sub('', line)
                line = line.strip()
                if stream is process.stdout:
                    print("STDOUT:", line, file=sys.stderr)
                    if "Loading world file" in line:
                        print("World file loading.", file=sys.stderr)
                        world_loaded = True
                elif stream is process.stderr:
                    print("STDERR:", line, file=sys.stderr)
                    if "Getting models from[http://models.gazebosim.org/]" in line:
                        print("Model downloading started.", file=sys.stderr)
                        models_downloading = True
            if not readable:
                dt = time.time() - start
                print(f"waiting for gazebo... time elapsed: {dt:.1f} sec.", file=sys.stderr)
                # Check if both conditions have been met
                if world_loaded and models_downloading:
                    print("Both conditions met. Preparing to shutdown gzserver.", file=sys.stderr)
                    time.sleep(2)  # Wait for 5 seconds
                    break

    except Exception as e:
        print(f"An error occurred: {e}", file=sys.stderr)
    finally:
        process.terminate()
        process.wait()  # Ensure all subprocess resources are cleaned up
        print("gzserver has been terminated.", file=sys.stderr)
        print("done", file=sys.stderr)

def wait_for_gzserver():
    run_gzserver()

if __name__ == "__main__":
    run_gzserver()