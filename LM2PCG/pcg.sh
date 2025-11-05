#!/bin/bash
# Simple wrapper to auto-build and run pcg_room

set -e  # Exit on error

# Check if input path is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 <input_path>"
    echo "Example: $0 ./data/rooms/Full\ House"
    exit 1
fi

# Get the script's directory (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BUILD_DIR="$SCRIPT_DIR/build"
PCG_ROOM_EXE="$BUILD_DIR/pcg_room"

# Check if pcg_room executable exists
if [ ! -f "$PCG_ROOM_EXE" ]; then
    echo "🔨 pcg_room not found. Building project..."
    
    # Create build directory if it doesn't exist
    mkdir -p "$BUILD_DIR"
    
    # Configure and build
    cd "$BUILD_DIR"
    cmake -DCMAKE_BUILD_TYPE=Release .. || {
        echo "❌ CMake configuration failed!"
        exit 1
    }
    
    cmake --build . -j || {
        echo "❌ Build failed!"
        exit 1
    }
    
    cd "$SCRIPT_DIR"
    echo "✅ Build completed successfully!"
    echo ""
fi

# Check if executable exists after build attempt
if [ ! -f "$PCG_ROOM_EXE" ]; then
    echo "❌ Error: pcg_room executable not found after build!"
    exit 1
fi

# Run pcg_room with the input path
echo "🚀 Running: pcg_room $1"
echo ""
"$PCG_ROOM_EXE" "$1"
