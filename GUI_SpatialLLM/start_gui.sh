#!/bin/bash
# Quick start script for SpatialLLM GUI

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "🚀 Starting SpatialLLM GUI..."

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found!"
    echo "   Please run: ./setup_gui.sh first"
    exit 1
fi

# Check if SAM2 model exists
SAM2_MODEL="../SAM23D/SAM2/checkpoints/sam2.1_hiera_large.pt"
if [ ! -f "$SAM2_MODEL" ]; then
    echo "⚠️  SAM2 model not found!"
    echo ""
    echo "📥 Downloading SAM2.1 Hiera Large model (~856MB)..."
    mkdir -p "../SAM23D/SAM2/checkpoints"
    curl -L -o "$SAM2_MODEL" https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt
    echo "✅ Model downloaded successfully!"
fi

# Start bridge server in background
echo ""
echo "📡 Starting bridge server on port 5056..."
source venv/bin/activate
python3 bridge_server_final.py &
BRIDGE_PID=$!

# Wait a moment for bridge server to start
sleep 2

# Check if bridge server is running
if ps -p $BRIDGE_PID > /dev/null; then
    echo "✅ Bridge server started (PID: $BRIDGE_PID)"
else
    echo "❌ Bridge server failed to start"
    exit 1
fi

# Start Streamlit
echo ""
echo "🌐 Starting Streamlit GUI on port 8501..."
echo "   Open your browser to: http://localhost:8501"
echo ""
echo "💡 Press Ctrl+C to stop both servers"
echo ""

# Trap Ctrl+C to kill bridge server
trap "echo '\n🛑 Stopping servers...'; kill $BRIDGE_PID 2>/dev/null; exit" INT TERM

streamlit run GUI_streamlit.py

# Cleanup
kill $BRIDGE_PID 2>/dev/null
