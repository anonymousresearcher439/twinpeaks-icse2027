#!/bin/bash

source /home/user/.profile
source /usr/share/gazebo-9/setup.sh

apt-get install --yes python
# Get Ardupilot Models

GZWEB_PATH=/usr/local/gzweb
# if given a path to gzweb, (using $1) use that instead 
if [ -n "$1" ]; then
    GZWEB_PATH="$1"
fi
cd "$GZWEB_PATH"


export GAZEBO_MODEL_PATH="/usr/local/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}"


# ARM CPU workaround: This change makes it so you can run gzweb and it's npm scripts on ARM-based systems.
# Without this change gzweb (and the other npm scripts) just crash.
#
# We fix the problem by reverting to an older version of the "websocket" library.
# If you're running on an ARM CPU, we need to use an older version of the
# "websocket" library because the newer versions depend on a native library that
# isn't built for Linux on ARM. Thankfully the older version is a pure JS
# library. Unfortunately, the old version might be a little slower but it's
# better than nothing. 
dpkgArch="$(dpkg --print-architecture)"
if [ "$dpkgArch" = "arm64" ]; then
    sed -i 's/"websocket": "\^1\.0\.25"/"websocket": "1.0.25"/g' package.json
fi

export PATH="/usr/local/node-v11.15.0-linux-x64/bin:/usr/local/node-v11.15.0-linux-arm64/bin:$PATH"
npm run deploy --- -m local -c -t
# npm run deploy --- -m -c -t