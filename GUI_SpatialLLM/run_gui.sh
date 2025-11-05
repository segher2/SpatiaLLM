#!/bin/bash

# Run Spatial LLM GUI
# This script starts the Streamlit web interface

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Activate virtual environment
VENV_PATH="../venv/bin/activate"
if [ -f "$VENV_PATH" ]; then
    echo "🔧 Activating virtual environment..."
    source "$VENV_PATH"
else
    echo "❌ Virtual environment not found at $VENV_PATH"
    echo "Please create it first with: python3 -m venv ../venv"
    exit 1
fi

# Check if dependencies are installed
if ! python3 -c "import streamlit" 2>/dev/null; then
    echo "📦 Installing GUI dependencies..."
    pip install flask==3.0.0 flask-cors==4.0.0 streamlit==1.28.0 streamlit-js-eval==0.1.5 pillow requests psutil
fi

# Check if database exists
DB_PATH="../LM2PCG/spatial_rooms.db"
if [ ! -f "$DB_PATH" ]; then
    echo "⚠️  Warning: Database not found at $DB_PATH"
    echo "You may need to populate the database first."
    echo ""
fi

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║           🚀 Starting Spatial LLM GUI                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "📍 GUI will open at: http://localhost:8501"
echo ""
echo "✨ To use the GUI:"
echo "   1. Click 'Start Session' in the sidebar"
echo "   2. Wait for 'Spatial AI Agent initialized' message"
echo "   3. Start asking questions in the chat!"
echo ""
echo "💡 Optional: Run bridge server for 3D segmentation"
echo "   Open another terminal and run:"
echo "   python bridge_server_final.py"
echo ""
echo "Press Ctrl+C to stop the server"
echo "════════════════════════════════════════════════════════════"
echo ""

# Start Streamlit
streamlit run GUI_streamlit.py
