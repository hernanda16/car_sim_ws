#!/bin/bash
# Simple script to setup models folder

# Lokasi script ini
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script location: $SCRIPT_DIR"

# Langsung ke src/towing
cd "$SCRIPT_DIR/src/towing" || { echo "Folder src/towing not found!"; exit 1; }

echo "Setting up models folder in $(pwd)..."

# Create folders
mkdir -p models/towing/meshes

# Copy STL files
cp meshes/*.STL models/towing/meshes/ 2>/dev/null

# Create model.config
cat > models/towing/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>towing</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
</model>
EOF

echo "Done! Models created at: $(pwd)/models/towing/"
ls -R models/