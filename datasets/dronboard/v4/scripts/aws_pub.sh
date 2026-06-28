
# Determine the directory of this script and cd into it (POSIX-compliant)
SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$SCRIPT_DIR"

mosquitto_pub \
  -h "a3dpdfmwa109lg-ats.iot.us-east-2.amazonaws.com" \
  -p 8883 \
  --cafile ../test/config/mqtt/CAs.crt \
  --cert ../test/config/mqtt/public.crt \
  --key ../test/config/mqtt/private.key \
  -i Navy \
  -t 'status_message' \
  -m 'hello from the shell script'
  # -q 0 \