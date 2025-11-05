#!/bin/bash
# Stop development servers

if [ -f /tmp/dev_servers.pid ]; then
    PIDS=$(cat /tmp/dev_servers.pid)
    echo "Stopping servers: $PIDS"
    kill $PIDS 2>/dev/null
    rm /tmp/dev_servers.pid
    echo "✓ Servers stopped"
else
    echo "No running servers found (no /tmp/dev_servers.pid)"
    echo "Killing any python api_server.py and npm dev processes..."
    pkill -f "api_server.py" 2>/dev/null
    pkill -f "vite" 2>/dev/null
    echo "Done"
fi
