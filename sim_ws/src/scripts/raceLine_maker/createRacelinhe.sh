#!/bin/bash

# ===== CONFIG =====
USER="arcus"
HOST="arcus-jetson.local"

REMOTE_DIR="/home/arcus/arcus/slam_map_saver/slam_maps/"
LOCAL_DIR="./maps"

PYTHON_SCRIPT="raceLine_generator.py"
RACELINE_FILE="./saved/waypoints.csv"
REMOTE_RACELINE_DIR="/home/arcus/arcus/resources/waypoints"

# ===== SETUP =====
mkdir -p "$LOCAL_DIR"



echo "Copying maps folder"
scp -r "$USER@$HOST:$REMOTE_DIR" .

if [ $? -ne 0 ]; then
    echo "Error copying directory"
    exit 1
fi

echo "Running Python script..."
python3 "$PYTHON_SCRIPT"

if [ $? -ne 0 ]; then
    echo "Python script failed"
    exit 1
fi

mv saved/raceline.csv $RACELINE_FILE
echo "Sending raceline back to remote..."
scp "$RACELINE_FILE" "$USER@$HOST:$REMOTE_RACELINE_DIR"

if [ $? -ne 0 ]; then
    echo "Error sending output file"
    exit 1
fi

echo "Done!"
