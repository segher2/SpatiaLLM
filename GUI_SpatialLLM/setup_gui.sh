#!/bin/bash
# Setup script for SpatialLLM GUI

set -e  # Exit on error

echo "🚀 Setting up SpatialLLM GUI..."

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo ""
echo "📁 Current directory: $SCRIPT_DIR"
echo "🐍 Python version: $(python3 --version)"

# Step 1: Create virtual environment if it doesn't exist
VENV_DIR="$SCRIPT_DIR/venv"
if [ ! -d "$VENV_DIR" ]; then
    echo ""
    echo "🔧 Step 1: Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "✅ Virtual environment created at $VENV_DIR"
else
    echo ""
    echo "✅ Virtual environment already exists at $VENV_DIR"
fi

# Activate virtual environment
echo ""
echo "🔌 Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Step 2: Install Python dependencies
echo ""
echo "📦 Step 2: Installing Python dependencies..."
pip install --upgrade pip
pip install -r requirements.txt

# Step 3: Create SAM2 checkpoints directory
echo ""
echo "📂 Step 3: Creating SAM2 checkpoints directory..."
SAM2_CHECKPOINTS="$SCRIPT_DIR/../SAM23D/SAM2/checkpoints"
mkdir -p "$SAM2_CHECKPOINTS"

# Step 4: Check if SAM2 model exists
SAM2_MODEL="$SAM2_CHECKPOINTS/sam2.1_hiera_large.pt"
if [ -f "$SAM2_MODEL" ]; then
    echo "✅ SAM2 model already exists: $SAM2_MODEL"
else
    echo "⚠️  SAM2 model not found!"
    echo ""
    echo "📥 Please download the SAM2.1 Hiera Large model:"
    echo "   cd $SAM2_CHECKPOINTS"
    echo "   curl -L -o sam2.1_hiera_large.pt https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt"
    echo ""
    echo "   (This is a ~856MB file and will take a few minutes)"
fi

# Step 5: Check if panorama images exist
echo ""
echo "🖼️  Step 5: Checking for panorama images..."
PANO_DIR="$SCRIPT_DIR/../data/input/panoramas/images"
if [ -d "$PANO_DIR" ]; then
    NUM_IMAGES=$(ls -1 "$PANO_DIR"/*.jpg 2>/dev/null | wc -l | tr -d ' ')
    if [ "$NUM_IMAGES" -gt 0 ]; then
        echo "✅ Found $NUM_IMAGES panorama images in $PANO_DIR"
    else
        echo "⚠️  No panorama images found in $PANO_DIR"
    fi
else
    echo "⚠️  Panorama directory not found: $PANO_DIR"
fi

# Step 6: Check for background image
echo ""
echo "🎨 Step 6: Checking for background image..."
BG_IMAGE="$SCRIPT_DIR/bg_streamlit.png"
if [ ! -f "$BG_IMAGE" ]; then
    echo "⚠️  Background image not found: $BG_IMAGE"
    echo "   Creating a placeholder..."
    # Create a simple placeholder image using ImageMagick or just skip
    touch "$BG_IMAGE"
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "🚀 To start the GUI:"
echo "   Terminal 1: source venv/bin/activate && python3 bridge_server_final.py"
echo "   Terminal 2: source venv/bin/activate && streamlit run GUI_streamlit.py"
echo ""
echo "📝 Don't forget to download the SAM2 model if not already done!"
echo ""
echo "💡 Quick start:"
echo "   ./start_gui.sh   # This will start both servers automatically"
