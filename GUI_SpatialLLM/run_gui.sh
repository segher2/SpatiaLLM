#!/bin/bash
# Simple script to run the GUI

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "🚀 Starting SpatialLLM GUI..."
echo ""

# Check if bridge server is already running
if lsof -i :5056 > /dev/null 2>&1; then
    echo "✅ Bridge server already running on port 5056"
else
    echo "📡 Starting bridge server..."
    source venv/bin/activate
    python3 bridge_server_final.py > bridge_server.log 2>&1 &
    BRIDGE_PID=$!
    sleep 2
    
    if ps -p $BRIDGE_PID > /dev/null; then
        echo "✅ Bridge server started (PID: $BRIDGE_PID)"
    else
        echo "❌ Bridge server failed to start - check bridge_server.log"
        exit 1
    fi
fi

echo ""
echo "🌐 Starting Streamlit GUI..."
echo "   Your browser will open automatically"
echo ""
echo "💡 Press Ctrl+C to stop"
echo ""

# Start Streamlit (this will block)
source venv/bin/activate
streamlit run GUI_streamlit.py
