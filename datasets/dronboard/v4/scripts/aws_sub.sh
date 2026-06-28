#!/bin/bash
# Determine the directory of this script and cd into it (POSIX-compliant)
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$SCRIPT_DIR"

CLIENT_ID="mmtesting"

mosquitto_sub \
  -h "a3dpdfmwa109lg-ats.iot.us-east-2.amazonaws.com" \
  -p 8883 \
  --cafile ../test/config/mqtt/CAs.crt \
  --cert ../test/config/mqtt/public.crt \
  --key ../test/config/mqtt/private.key \
  -i $CLIENT_ID \
  -t 'sade_manager/#'

# Expected output:
# {"droneID": "DRONE-POLKADOT-1001"}
# {"pilotID": "PILOT-POLKADOT-201", "model_name": "DJI Mavic 3", "owner": "FA23456789"}
# {"pilotID": "PILOT-POLKADOT-201", "drone_id": "DRONE-POLKADOT-1001", "sade_zone_id": "Chain-Lake-SADE", "mission": "NASA demo mission"}

# export RESPONSE='{
#   "droneID": "DRONE-POLKADOT-1001",
#   "pilotID": "PILOT-POLKADOT-201",
#   "sade_zone_id": "Chain-Lake-SADE",
#   "decision": "PROVING_GROUND"
# }'
# echo "Publishing response: $RESPONSE"
# mosquitto_pub \
#   -h "a3dpdfmwa109lg-ats.iot.us-east-2.amazonaws.com" \
#   -p 8883 \
#   --cafile ../test/config/mqtt/CAs.crt \
#   --cert ../test/config/mqtt/public.crt \
#   --key ../test/config/mqtt/private.key \
#   -i $CLIENT_ID \
#   -t 'drone/Polkadot/sade_entry_response' \
#   -m "$RESPONSE"