#!/bin/bash

# ===== CONFIG =====
USER="arcus"
HOST="arcus-jetson.local"

MAP_FILE=""

REMOTE_MAP_DIR="/home/arcus/arcus/slam_map_saver/slam_maps"

echo "Sending raceline back to remote..."
scp "$RACELINE_FILE" "$USER@$HOST:$REMOTE_RACELINE_DIR"

if [ $? -ne 0 ]; then
    echo "Error sending output file"
    exit 1
fi

echo "Done!"

