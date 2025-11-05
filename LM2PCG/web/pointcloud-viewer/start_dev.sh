#!/bin/bash
# Start both frontend and backend servers for development

# Colors
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting development servers...${NC}"
echo -e "${CYAN}Frontend: http://localhost:5173/${NC}"
echo -e "${CYAN}Backend API: http://localhost:8090/${NC}"
echo ""

# Get the script directory (web/pointcloud-viewer)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Project root is two levels up
PROJECT_ROOT="$SCRIPT_DIR/../.."

# Start API server in background
echo -e "${CYAN}[API]${NC} Starting API server..."
cd "$PROJECT_ROOT"
python3 scripts/api_server.py --port 8090 > /tmp/api_server.log 2>&1 &
API_PID=$!
echo -e "${CYAN}[API]${NC} Started with PID: $API_PID"

# Wait for API server to start
sleep 2

# Start Vite dev server in background
echo -e "${CYAN}[VITE]${NC} Starting frontend dev server..."
cd "$SCRIPT_DIR"
npm run dev > /tmp/vite_server.log 2>&1 &
VITE_PID=$!
echo -e "${CYAN}[VITE]${NC} Started with PID: $VITE_PID"

# Wait for servers to start
sleep 3

echo ""
echo -e "${GREEN}✓ Both servers are running!${NC}"
echo -e "${YELLOW}View logs:${NC}"
echo "  API:   tail -f /tmp/api_server.log"
echo "  Vite:  tail -f /tmp/vite_server.log"
echo ""
echo -e "${YELLOW}To stop servers:${NC}"
echo "  kill $API_PID $VITE_PID"
echo ""
echo "PIDs saved to /tmp/dev_servers.pid"
echo "$API_PID $VITE_PID" > /tmp/dev_servers.pid
