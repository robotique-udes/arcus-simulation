#!/bin/bash

# ===== CONFIG =====
USER="arcus"
HOST="arcus-jetson.local"

RACELINE_FILE="saved/speed_zones.csv"
REMOTE_RACELINE_DIR="/home/arcus/arcus/resources/waypoints"

echo "Sending raceline back to remote..."
scp "$RACELINE_FILE" "$USER@$HOST:$REMOTE_RACELINE_DIR"

if [ $? -ne 0 ]; then
    echo "Error sending output file"
    exit 1
fi

echo "Done!"

