#!/bin/bash
# This script is used to debug the airleasing process by collecting all messages going in or out of dr_onboard

# create the output file name
prefix="tmp/airlese-bug-`date -I`"
n=1
while [[ -e ${prefix}-${n}.csv ]]; do
  ((n++))
done
outfile="${prefix}-${n}.csv"

host="host.docker.internal"

python scripts/mqtt_to_csv.py --output "$outfile" $host  'drone/+/airlease/status' 'airlease/request'