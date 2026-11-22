#!/bin/bash

# ===== CONFIG =====
USER="arcus"
HOST="arcus-jetson.local"
REMOTE_MAP_DIR="/home/arcus/arcus/slam_map_saver/slam_maps"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_MAP_DIR="$SCRIPT_DIR/slam_maps"

# ===== TROUVER LA CLEAN MAP LA PLUS RECENTE =====
BEST_TS=""
YAML_FILE=""

for f in "$LOCAL_MAP_DIR"/*_clean.yaml; do
    [ -e "$f" ] || continue                  # aucun fichier ne correspond au motif
    [ -f "${f%.yaml}.pgm" ] || continue      # il faut la paire yaml + pgm

    if [[ "$(basename "$f")" =~ ([0-9]{8}_[0-9]{6}) ]]; then
        ts="${BASH_REMATCH[1]}"
        if [[ "$ts" > "$BEST_TS" ]]; then
            BEST_TS="$ts"
            YAML_FILE="$f"
        fi
    fi
done

if [ -z "$YAML_FILE" ]; then
    echo "Aucune clean map (*_clean.yaml + .pgm) trouvée dans $LOCAL_MAP_DIR."
    echo "Lance d'abord 'Clean map'."
    exit 1
fi

PGM_FILE="${YAML_FILE%.yaml}.pgm"

echo "Sending clean map to remote..."
echo "  $(basename "$YAML_FILE")"
echo "  $(basename "$PGM_FILE")"

scp "$YAML_FILE" "$PGM_FILE" "$USER@$HOST:$REMOTE_MAP_DIR/"

if [ $? -ne 0 ]; then
    echo "Error sending clean map"
    exit 1
fi

echo "Done!"