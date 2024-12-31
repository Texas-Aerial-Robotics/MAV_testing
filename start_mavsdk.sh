#!/bin/sh

MAVSDK_PATH="venv/lib/python3.8/site-packages/mavsdk/bin"

# Kill any running instances of mavsdk_server
killall mavsdk_server 2>/dev/null || echo "No mavsdk_server processes to kill."

# Start the MAVSDK servers
"$MAVSDK_PATH/mavsdk_server" -p 50051 udp://:14541 &
"$MAVSDK_PATH/mavsdk_server" -p 50052 udp://:14542 &
