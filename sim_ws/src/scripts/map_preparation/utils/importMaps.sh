#!/bin/bash

# ===== CONFIG =====
USER="arcus"
HOST="arcus-jetson.local"

REMOTE_DIR="/home/arcus/arcus/slam_map_saver/slam_maps/"


echo "Copying maps folder"
scp -r "$USER@$HOST:$REMOTE_DIR" .

if [ $? -ne 0 ]; then
    echo "Error copying directory"
    exit 1
fi