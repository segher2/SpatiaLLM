#!/usr/bin/env python3 -u
"""
HTTP API server for the pointcloud viewer to resolve source file paths.

Provides REST endpoints:
- GET /api/resolve-object?code=<object_code>  - Resolve object by code (e.g., 0-7-12)
- GET /api/resolve-room?code=<room_code>      - Resolve room by code (e.g., 0-7)
- POST /api/submit-selection                  - Receive user selections from viewer

Usage:
    python scripts/api_server.py [--port 8080] [--host 0.0.0.0]
"""
import argparse
import json
import sys
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from pathlib import Path

# Force unbuffered output for real-time logging
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

# Add parent directory to path to import ai_api
sys.path.insert(0, str(Path(__file__).parent))

try:
    from ai_api import Dispatcher, parse_object_code
except ImportError as e:
    print(f"Error: Could not import ai_api.py. Make sure it's in the same directory.")
    print(f"Details: {e}")
    sys.exit(1)


class APIHandler(BaseHTTPRequestHandler):
    """HTTP request handler for the API"""
    
    def __init__(self, *args, dispatcher: Dispatcher = None, **kwargs):
        self.dispatcher = dispatcher or Dispatcher()
        super().__init__(*args, **kwargs)
    
    def do_OPTIONS(self):
        """Handle CORS preflight"""
        self.send_response(200)
        self._send_cors_headers()
        self.end_headers()
    
    def do_GET(self):
        """Handle GET requests"""
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)
        
        if path == '/api/resolve-object':
            self._handle_resolve_object(params)
        elif path == '/api/resolve-room':
            self._handle_resolve_room(params)
        elif path == '/api/download-file':
            self._handle_download_file(params)
        elif path == '/health' or path == '/api/health':
            self._handle_health()
        else:
            self._send_error(404, f"Unknown endpoint: {path}")
    
    def do_POST(self):
        """Handle POST requests"""
        parsed = urlparse(self.path)
        path = parsed.path
        
        if path == '/api/submit-selection':
            self._handle_submit_selection()
        else:
            self._send_error(404, f"Unknown endpoint: {path}")
    
    def _send_cors_headers(self):
        """Send CORS headers"""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
    
    def _send_json(self, data: dict, status: int = 200):
        """Send JSON response"""
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self._send_cors_headers()
        self.end_headers()
        self.wfile.write(json.dumps(data, indent=2).encode('utf-8'))
    
    def _send_error(self, status: int, message: str):
        """Send error response"""
        self._send_json({
            'success': False,
            'error': message
        }, status)
    
    def _handle_health(self):
        """Health check endpoint"""
        self._send_json({
            'success': True,
            'status': 'ok',
            'service': 'pointcloud-api',
            'index_loaded': bool(self.dispatcher.index)
        })
    
    def _handle_submit_selection(self):
        """Handle POST /api/submit-selection - User confirms selection in viewer"""
        try:
            # Read request body
            content_length = int(self.headers.get('Content-Length', 0))
            if content_length == 0:
                self._send_error(400, 'Empty request body')
                return
            
            body = self.rfile.read(content_length)
            payload = json.loads(body.decode('utf-8'))
            
            # Support two formats:
            # 1. New format with session_id: {"session_id": "...", "selections": [...]}
            # 2. Legacy format: [...] (array directly)
            session_id = None
            if isinstance(payload, dict):
                session_id = payload.get('session_id')
                data = payload.get('selections', [])
            elif isinstance(payload, list):
                data = payload
            else:
                self._send_error(400, 'Invalid request format')
                return
            
            # Validate data structure
            if not isinstance(data, list):
                self._send_error(400, 'Expected array of selected objects')
                return
            
            # Print friendly real-time message to terminal
            print("\n" + "="*60)
            if session_id:
                print(f"🎯 实时检测到用户选择！(Session: {session_id})")
            else:
                print("🎯 实时检测到用户选择！")
            print("="*60)
            
            if len(data) == 0:
                print("📭 用户清空了选择")
            else:
                print(f"✅ 用户选择了 {len(data)} 个物体：")
                print()
                for i, item in enumerate(data, 1):
                    display_name = item.get('displayName', 'unknown')
                    item_code = item.get('itemCode', 'unknown')
                    print(f"   {i}. {display_name} ({item_code})")
                
                print()
                print("📦 详细信息：")
                print(json.dumps(data, indent=2, ensure_ascii=False))
            
            print("="*60)
            print()
            
            # Save selection to file for AI agent
            # If session_id is provided, save to session-specific file
            if session_id:
                selection_file = Path(f"/tmp/viewer_selection_{session_id}.json")
                print(f"💾 选择已保存到: {selection_file} (session-specific)")
            else:
                selection_file = Path("/tmp/viewer_selection.json")
                print(f"💾 选择已保存到: {selection_file} (legacy)")
            
            try:
                with open(selection_file, 'w', encoding='utf-8') as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                print(f"✅ 文件写入成功\n")
            except Exception as e:
                print(f"⚠️  保存选择文件失败: {e}\n")
            
            # Send success response
            self._send_json({
                'success': True,
                'message': f'Received selection of {len(data)} objects',
                'count': len(data)
            })
            
        except json.JSONDecodeError as e:
            self._send_error(400, f'Invalid JSON: {str(e)}')
        except Exception as e:
            self._send_error(500, f'Server error: {str(e)}')
    
    def _handle_resolve_object(self, params: dict):
        """Handle /api/resolve-object?code=<object_code>"""
        code = params.get('code', [None])[0]
        if not code:
            self._send_error(400, 'Missing required parameter: code')
            return
        
        try:
            # Parse object code
            floor_id, room_id, object_id = parse_object_code(code)
            
            # Find assets using PathIndex
            assets = self.dispatcher.index.find_assets(code)
            
            if not assets:
                self._send_error(404, f'Object not found: {code}')
                return
            
            # Get cluster, uobb, and mesh paths
            cluster_path = assets.clusters[0] if assets.clusters else None
            uobb_path = assets.uobbs[0] if assets.uobbs else None
            mesh_path = assets.meshes[0] if assets.meshes else None
            
            # Convert paths to relative paths (relative to project root)
            project_root = Path(__file__).parent.parent.resolve()
            
            def to_relative(p: Path) -> str:
                try:
                    return str(p.relative_to(project_root))
                except ValueError:
                    # If path is not relative to project root, return absolute
                    return str(p)
            
            # Try to find CSV data
            csv_files = self.dispatcher.index.find_csv(floor_id, room_id)
            csv_data = None
            if csv_files:
                # Parse CSV to find this object's row
                import csv
                try:
                    with open(csv_files[0], 'r') as f:
                        reader = csv.DictReader(f)
                        for row in reader:
                            if row.get('object_code') == code:
                                csv_data = dict(row)
                                break
                except Exception:
                    pass
            
            # Build response
            response = {
                'success': True,
                'data': {
                    'object_code': code,
                    'floor_id': floor_id,
                    'room_id': room_id,
                    'object_id': object_id,
                    'class': csv_data.get('class') if csv_data else None,
                    'cluster_path': to_relative(cluster_path) if cluster_path else None,
                    'uobb_path': to_relative(uobb_path) if uobb_path else None,
                    'mesh_path': to_relative(mesh_path) if mesh_path else None,
                    'room_dir': str(assets.room_dir) if assets.room_dir else None,
                    'csv_data': csv_data
                }
            }
            
            self._send_json(response)
            
        except ValueError as e:
            self._send_error(400, str(e))
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_error(500, f'Internal error: {str(e)}')
    
    def _handle_resolve_room(self, params: dict):
        """Handle /api/resolve-room?code=<room_code>"""
        code = params.get('code', [None])[0]
        if not code:
            self._send_error(400, 'Missing required parameter: code')
            return
        
        try:
            # Parse room code (format: "0-7" or "0_7")
            parts = code.replace('_', '-').split('-')
            if len(parts) != 2:
                raise ValueError(f'Invalid room code format: {code}. Expected format: <floor>-<room>')
            floor_id = int(parts[0])
            room_id = int(parts[1])
            
            # Find CSV
            csv_files = self.dispatcher.index.find_csv(floor_id, room_id)
            csv_path = csv_files[0] if csv_files else None
            
            # Find shell and uobb
            shell_paths, uobb_paths = self.dispatcher.index.find_room_shells(floor_id, room_id)
            shell_path = shell_paths[0] if shell_paths else None
            shell_uobb_path = uobb_paths[0] if uobb_paths else None
            
            # Try to find clusters directory
            clusters_dir = None
            if csv_path:
                room_dir = csv_path.parent
                potential_clusters = room_dir / 'results' / 'filtered_clusters'
                if potential_clusters.exists():
                    clusters_dir = potential_clusters
            
            # Convert paths to relative paths (relative to project root)
            project_root = Path(__file__).parent.parent.resolve()
            
            def to_relative(p: Path) -> str:
                try:
                    return str(p.relative_to(project_root))
                except ValueError:
                    # If path is not relative to project root, return absolute
                    return str(p)
            
            # Build response
            response = {
                'success': True,
                'data': {
                    'room_code': code,
                    'floor_id': floor_id,
                    'room_id': room_id,
                    'csv_path': to_relative(csv_path) if csv_path else None,
                    'shell_path': to_relative(shell_path) if shell_path else None,
                    'shell_uobb_path': to_relative(shell_uobb_path) if shell_uobb_path else None,
                    'clusters_dir': to_relative(clusters_dir) if clusters_dir else None
                }
            }
            
            self._send_json(response)
            
        except ValueError as e:
            self._send_error(400, str(e))
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_error(500, f'Internal error: {str(e)}')
    
    def _handle_download_file(self, params: dict):
        """Handle /api/download-file?path=<file_path>"""
        file_path_str = params.get('path', [None])[0]
        if not file_path_str:
            self._send_error(400, 'Missing required parameter: path')
            return
        
        try:
            # Get project root
            project_root = Path(__file__).parent.parent.resolve()
            
            # If path is relative, resolve it relative to project root
            file_path = Path(file_path_str)
            if not file_path.is_absolute():
                file_path = project_root / file_path
            
            file_path = file_path.resolve()
            
            # Security check: ensure the file is within the project directory
            if not str(file_path).startswith(str(project_root)):
                self._send_error(403, 'Access denied: file is outside project directory')
                return
            
            if not file_path.exists():
                self._send_error(404, f'File not found: {file_path_str}')
                return
            
            if not file_path.is_file():
                self._send_error(400, f'Not a file: {file_path_str}')
                return
            
            # Send file
            self.send_response(200)
            self.send_header('Content-Type', 'application/octet-stream')
            self.send_header('Content-Disposition', f'attachment; filename="{file_path.name}"')
            self._send_cors_headers()
            
            # Get file size
            file_size = file_path.stat().st_size
            self.send_header('Content-Length', str(file_size))
            self.end_headers()
            
            # Stream file content
            with open(file_path, 'rb') as f:
                chunk_size = 8192
                while True:
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            self._send_error(500, f'Internal error: {str(e)}')
    
    def log_message(self, format, *args):
        """Override to customize logging"""
        print(f"[API] {self.address_string()} - {format % args}")


def create_handler_class(dispatcher: Dispatcher):
    """Factory function to create handler with dispatcher"""
    return lambda *args, **kwargs: APIHandler(*args, dispatcher=dispatcher, **kwargs)


def main():
    parser = argparse.ArgumentParser(description='HTTP API server for pointcloud viewer')
    parser.add_argument('--port', type=int, default=8080, help='Port to listen on (default: 8080)')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to (default: 0.0.0.0)')
    args = parser.parse_args()
    
    # Initialize dispatcher
    print('[API] Initializing dispatcher and loading index...')
    try:
        dispatcher = Dispatcher()
        print(f'[API] Index loaded: {len(dispatcher.index.by_filename)} files, {len(dispatcher.index.assets_by_object)} objects')
    except Exception as e:
        print(f'[API] Warning: Failed to load index: {e}')
        print('[API] Continuing with empty dispatcher...')
        dispatcher = Dispatcher()
    
    # Create server
    handler_class = create_handler_class(dispatcher)
    server = HTTPServer((args.host, args.port), handler_class)
    
    print(f'\n[API] ✓ Server started at http://{args.host}:{args.port}/')
    print(f'[API] Endpoints:')
    print(f'[API]   GET /api/resolve-object?code=<object_code>')
    print(f'[API]   GET /api/resolve-room?code=<room_code>')
    print(f'[API]   GET /api/download-file?path=<file_path>')
    print(f'[API]   GET /health')
    print(f'[API] Press Ctrl+C to stop\n')
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[API] Shutting down server...')
        server.shutdown()


if __name__ == '__main__':
    main()
