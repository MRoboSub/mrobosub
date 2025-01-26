#!/bin/sh

# Default values
DEFAULT_MAX_SIZE=$((10*1024*1024))  # Default max file size: 10 MB
DEFAULT_OUT_FILE="log.txt"
DEFAULT_SLEEP_RATE=1               # Default sleep rate: 1 second
DEFAULT_TOPIC="/target_pose/heave" # Default ROS topic

# Parse arguments
while getopts "s:o:r:t:h" opt; do
    case $opt in
        s) MAX_SIZE=$(($OPTARG * 1024 * 1024)) ;;  # Convert MB to bytes
        o) OUT_FILE="$OPTARG" ;;
        r) SLEEP_RATE="$OPTARG" ;;                # Sleep rate in seconds
        t) TOPIC="$OPTARG" ;;                     # ROS topic
        h) 
            echo "Usage: $0 [-s max_size_in_MB] [-o output_file_name] [-r sleep_rate_in_seconds] [-t ros_topic]"
            echo "  -s: Set the maximum file size in MB (default: 10 MB)"
            echo "  -o: Set the output file name (default: output.txt)"
            echo "  -r: Set the sleep rate in seconds for file size checking (default: 1 second)"
            echo "  -t: Set the ROS topic to echo (default: /target_pose/heave)"
            exit 0
            ;;
        *) 
            echo "Invalid option. Use -h for help."
            exit 1
            ;;
    esac
done

# Use defaults if parameters are not set
MAX_SIZE=${MAX_SIZE:-$DEFAULT_MAX_SIZE}
OUT_FILE=${OUT_FILE:-$DEFAULT_OUT_FILE}
SLEEP_RATE=${SLEEP_RATE:-$DEFAULT_SLEEP_RATE}
TOPIC=${TOPIC:-$DEFAULT_TOPIC}

# Start the rostopic echo command in the background
rostopic echo "$TOPIC" > "$OUT_FILE" &

# Get the process ID of the background rostopic command
SCRIPT_PID=$!

# Monitor file size and rotate log files
while true; do
    if [ -f "$OUT_FILE" ] && [ $(stat -c%s "$OUT_FILE") -ge $MAX_SIZE ]; then
        mv "$OUT_FILE" "$OUT_FILE.$(date +%s)"
    fi
    sleep "$SLEEP_RATE"
done

# Wait for the rostopic command to finish
wait $SCRIPT_PID