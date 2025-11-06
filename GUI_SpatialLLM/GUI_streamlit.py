import subprocess
import psutil
import os
import re
import glob
import base64
import json
from PIL import Image
import streamlit as st
import streamlit.components.v1 as components
import requests
import time
from streamlit_js_eval import streamlit_js_eval
import mutli_room_agent2 as room_agent
import ai_api_wrapper as ai_wrapper
import enrich_room_types as enrich_rooms
import room_database


# Page config
st.set_page_config(page_title="Spatial Understanding via Multi-Modal LLM", layout="wide")

# Clean SAM23D/outputs directory on startup
if 'outputs_cleaned' not in st.session_state:
    import shutil
    from pathlib import Path
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(current_dir)
    outputs_dir = Path(root_dir) / "SAM23D" / "outputs"
    
    if outputs_dir.exists():
        shutil.rmtree(outputs_dir)
        outputs_dir.mkdir(parents=True, exist_ok=True)
    
    st.session_state.outputs_cleaned = True

# Styling
st.markdown(
    """
    <style>
    section[data-testid="stAppViewContainer"] { color: white !important; }
    section[data-testid="stAppViewContainer"] * { color: white !important; }
    section[data-testid="stAppViewContainer"] div[data-testid="stNotification"] {
        background: var(--background-color, rgba(0,0,0,0.25)) !important;
        border: 1px solid var(--secondary-background-color, rgba(255,255,255,0.2)) !important;
        color: white !important;
    }
    .st-emotion-cache-1an99fx, .st-emotion-cache-3uj0rx { color: #fff !important; }
    .st-emotion-cache-1iitq1e, .st-emotion-cache-1fee4w7, .st-dy { background: rgba(240,242,246,0.1) !important; color: #fff !important; }
    </style>
    """,
    unsafe_allow_html=True
)


# Background image
def add_bg_from_local(image_file):
    try:
        with open(image_file, "rb") as f:
            encoded = base64.b64encode(f.read()).decode()
        st.markdown(
            f"""
            <style>
            .stApp {{
                background-image: url("data:image/png;base64,{encoded}");
                background-attachment: fixed;
                background-size: cover;
            }}
            </style>
            """,
            unsafe_allow_html=True
        )
    except FileNotFoundError:
        st.markdown("<style>.stApp { background: #0e1117; }</style>", unsafe_allow_html=True)


# Use absolute path for background image
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
add_bg_from_local(os.path.join(SCRIPT_DIR, "bg_streamlit.png"))

# Initialize states
for key, val in {
    "agent": None,
    "chat_ui": [],
    "page": "main",
    "pano_idx": 0,
    "clicked_points": [],
    "overlay_url": None,
    "yaw": 0,
    "pitch": 0,
    "hfov": 90
}.items():
    if key not in st.session_state:
        st.session_state[key] = val

# Sidebar
st.sidebar.markdown("### Session Controls")

start_btn = st.sidebar.button("Start Session", use_container_width=True)
restart_btn = st.sidebar.button("Restart Session", use_container_width=True)
end_btn = st.sidebar.button("End Session", use_container_width=True)
virtual_btn = st.sidebar.button("Virtual Mode", use_container_width=True)

# Logic for sidebar buttons
if start_btn:
    if st.session_state.agent:
        st.warning("Session already active.")
    else:
        with st.spinner("Initializing Spatial AI Agent..."):
            try:
                exec("room_database")
                exec("enrich_rooms")
                exec("ai_wrapper")
                st.session_state.agent = room_agent
                st.session_state.chat_ui = []
                st.success("Spatial AI Agent initialized.")
            except Exception as e:
                st.error(f"Failed to start AI Agent: {e}")

elif restart_btn:
    with st.spinner("Restarting AI Agent..."):
        try:
            exec("room_database")
            exec("enrich_rooms")
            exec("ai_wrapper")
            st.session_state.agent = room_agent
            st.session_state.chat_ui = []
            st.success("Agent restarted successfully!")
        except Exception as e:
            st.error(f"Restart failed: {e}")

elif end_btn:
    st.sidebar.warning("🛑 Preparing to end session...")

    countdown_placeholder = st.empty()
    for i in range(3, 0, -1):
        countdown_placeholder.info(f"Closing in {i}...")
        time.sleep(1)

    countdown_placeholder.empty()
    st.success("""
    ## Session completed successfully!

    **Next steps:**
    - You may close this browser tab
    - Or click below to go back to the home page
    """)

    if st.button("Go back to home"):
        st.rerun()

    st.stop()

elif virtual_btn:
    st.session_state.page = "virtual"
    st.rerun()

# Bridge server
BRIDGE_SERVER_URL = "http://localhost:5056"


def check_bridge_server():
    try:
        response = requests.get(f"{BRIDGE_SERVER_URL}/health", timeout=2)
        return response.status_code == 200
    except:
        return False


def is_port_in_use(port: int) -> bool:
    for conn in psutil.net_connections():
        if conn.laddr.port == port:
            return True
    return False


# Header
st.markdown('<h1 style="color:white;">Spatial LLM</h1>', unsafe_allow_html=True)
st.markdown('<p style="color:white;">Bridging The Gap Between Natural Language and 3D Scans</p>',
            unsafe_allow_html=True)

if not check_bridge_server():
    st.error("""
    **Bridge Server Not Found**
    Run it first:
    ```bash
    python bridge_server_final.py
    ```
    """)
    st.stop()

# Panorama setup + virtual mode page
if st.session_state.page == "virtual":
    # Use absolute path based on script location (SCRIPT_DIR already defined above)
    PANOS_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "data", "input", "panoramas", "images")
    pano_files = sorted(
        glob.glob(os.path.join(PANOS_DIR, "*.jpg"))
        + glob.glob(os.path.join(PANOS_DIR, "*.jpeg"))
        + glob.glob(os.path.join(PANOS_DIR, "*.png"))
    )


    def _natural_key(s: str):
        return [int(t) if t.isdigit() else t.lower() for t in re.split(r'(\d+)', os.path.basename(s))]


    pano_files.sort(key=_natural_key)

    if not pano_files:
        st.warning(f"No panoramas found in {PANOS_DIR}")
        st.stop()

    # Session state
    if "pano_idx" not in st.session_state:
        st.session_state.pano_idx = 0
    if "clicked_points" not in st.session_state:
        st.session_state.clicked_points = []
    if "click_mode" not in st.session_state:
        st.session_state.click_mode = False
    if "max_points_reached" not in st.session_state:
        st.session_state.max_points_reached = False
    if "overlay_url" not in st.session_state:
        st.session_state.overlay_url = None
    if "viewer_key" not in st.session_state:
        st.session_state.viewer_key = 0
    if "yaw" not in st.session_state:
        st.session_state.yaw = 0
    if "pitch" not in st.session_state:
        st.session_state.pitch = 0
    if "hfov" not in st.session_state:
        st.session_state.hfov = 90
    if "processing_complete" not in st.session_state:
        st.session_state.processing_complete = False
    if "las_path" not in st.session_state:
        st.session_state.las_path = None


    def wrap_idx(i: int, n: int) -> int:
        return (i % n) if n else 0


    def file_to_data_url(path: str) -> str:
        with open(path, "rb") as f:
            raw = f.read()
        ext = os.path.splitext(path)[1].lower()
        mime = "image/jpeg" if ext in (".jpg", ".jpeg") else "image/png"
        return f"data:{mime};base64,{base64.b64encode(raw).decode()}"


    def pitch_yaw_to_pixel(pitch: float, yaw: float, width: int, height: int):
        yaw_norm = (yaw + 360.0) % 360.0
        x = yaw_norm / 360.0 * width
        y = (90.0 - pitch) / 180.0 * height
        return int(round(x)), int(round(y))


    # Pannellum renderer
    def render_pannellum_with_clicks(img_data_url, existing_points=None, height=560, image_filename=None):
        existing_points = (existing_points or [])[:5]
        points_json = json.dumps(existing_points)
        num_points = len(existing_points)
        filename_json = json.dumps(image_filename or "")
        click_mode = "true" if st.session_state.click_mode else "false"

        html = f"""
        <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/pannellum@2.5.6/build/pannellum.css"/>
        <div style="position:relative;">
          <div id="panoViewer" style="width:100%;height:{height}px;border-radius:8px;overflow:hidden;"></div>
          <div id="loadingOverlay" style="display:none;position:absolute;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.7);z-index:1000;border-radius:8px;">
            <div style="position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);text-align:center;color:white;">
              <div style="font-size:48px;margin-bottom:20px;">⏳</div>
              <div style="font-size:24px;font-weight:bold;">Processing segmentation...</div>
              <div style="font-size:16px;margin-top:10px;opacity:0.8;">This may take a few seconds</div>
              <div style="margin-top:20px;">
                <div class="spinner"></div>
              </div>
            </div>
          </div>
          <div id="showResultsButton" style="display:none;position:absolute;bottom:20px;left:50%;transform:translateX(-50%);z-index:999;">
            <button onclick="showResults()" style="background:#4CAF50;color:white;border:none;padding:15px 30px;font-size:18px;font-weight:bold;border-radius:8px;cursor:pointer;box-shadow:0 4px 6px rgba(0,0,0,0.3);transition:all 0.3s;">
              🔍 Show Results
            </button>
          </div>
        </div>
        <script src="https://cdn.jsdelivr.net/npm/pannellum@2.5.6/build/pannellum.js"></script>
        <script>
          // Function to open viewer with latest results
          function showResults() {{
            console.log("🔍 Show Results button clicked");
            // Call the Python backend to run visualize_latest.py
            fetch("{BRIDGE_SERVER_URL}/show_results", {{
              method: "POST",
              headers: {{ "Content-Type": "application/json" }}
            }})
            .then(response => response.json())
            .then(data => {{
              if (data.url) {{
                console.log("Opening viewer at:", data.url);
                window.open(data.url, '_blank');
              }} else if (data.error) {{
                alert("Error: " + data.error);
              }}
            }})
            .catch(err => {{
              console.error("Error showing results:", err);
              alert("Failed to open viewer: " + err.message);
            }});
          }}
          
          const viewer = pannellum.viewer("panoViewer", {{
            type: "equirectangular",
            panorama: "{img_data_url}",
            autoLoad: true,
            pitch: {st.session_state.pitch},
            yaw: {st.session_state.yaw},
            hfov: {st.session_state.hfov},
            haov: 360, vaov: 180,
            showZoomCtrl: true, compass: false
          }});
          let enableClick = {click_mode};
          let clickedPoints = {points_json};
          let numPoints = {num_points};
          let hotspotIds = [];

          clickedPoints.forEach((p, i) => {{
            const id = "hotspot_" + i;
            viewer.addHotSpot({{
              id: id, pitch: p.pitch, yaw: p.yaw,
              type: "info",
              text: `Point ${{i+1}}: Pitch ${{p.pitch.toFixed(2)}}°, Yaw ${{p.yaw.toFixed(2)}}°`,
              cssClass: "custom-hotspot"
            }});
            hotspotIds.push(id);
          }});

          let isDragging = false;
          let dragStartTime = 0;

          viewer.on('mousedown', e => {{ isDragging = false; dragStartTime = Date.now(); }});
          viewer.on('mousemove', e => {{ if (Date.now() - dragStartTime > 100) isDragging = true; }});

          viewer.on('mouseup', function(e) {{
              if (isDragging || (Date.now() - dragStartTime) > 200) return;

              // Right-click → reset all points
              if (e.button === 2) {{
                console.log("Right-click detected → resetting all points");

                // Remove all existing hotspots
                hotspotIds.forEach(id => viewer.removeHotSpot(id));
                hotspotIds = [];
                clickedPoints = [];
                numPoints = 0;

                // Notify backend to reset click data
                fetch("{BRIDGE_SERVER_URL}/click/reset", {{
                  method: "POST"
                }})
                  .then(() => {{
                    console.log("Backend clicks reset");

                  }})
                  .catch(err => console.error("Error resetting points:", err));

                return;
              }}

              // Left-click → add new point
              if (!enableClick) return;
              if (e.button === 0) {{
                if (numPoints >= 5) {{
                  console.warn("Maximum 5 points reached");
                  return;
                }}

                const coords = viewer.mouseEventToCoords(e);
                if (coords && coords.length === 2) {{
                  const pitch = parseFloat(coords[0].toFixed(4));
                  const yaw = parseFloat(coords[1].toFixed(4));

                  const id = "hotspot_" + numPoints;
                  viewer.addHotSpot({{
                    id: id,
                    pitch: pitch,
                    yaw: yaw,
                    type: 'info',
                    text: `Point ${{numPoints + 1}}: Pitch ${{pitch}}°, Yaw ${{yaw}}°`,
                    cssClass: 'custom-hotspot'
                  }});

                  hotspotIds.push(id);
                  numPoints++;
                  clickedPoints.push({{ pitch, yaw }});

                  // Show loading overlay if this is the 5th point
                  if (numPoints === 5) {{
                    document.getElementById('loadingOverlay').style.display = 'block';
                    console.log("🔄 Starting SAM2 processing...");
                  }}

                  fetch("{BRIDGE_SERVER_URL}/click", {{
                    method: "POST",
                    headers: {{ "Content-Type": "application/json" }},
                    body: JSON.stringify({{ pitch, yaw, point_number: numPoints, image_filename: {filename_json} }})
                  }})
                    .then(r => {{
                      console.log("Response status:", r.status);
                      if (!r.ok) {{
                        throw new Error(`HTTP error! status: ${{r.status}}`);
                      }}
                      return r.json();
                    }})
                    .then(data => {{
                      console.log("Server response:", data);
                      console.log("sam2_auto_run:", data.sam2_auto_run);
                      console.log("sam2_result exists:", !!data.sam2_result);
                      if (data.sam2_result) {{
                        console.log("overlay_base64 exists:", !!data.sam2_result.overlay_base64);
                        console.log("overlay_base64 length:", data.sam2_result.overlay_base64 ? data.sam2_result.overlay_base64.length : 0);
                        console.log("las_path exists:", !!data.sam2_result.las_path);
                      }}
                      
                      // Hide loading overlay
                      document.getElementById('loadingOverlay').style.display = 'none';
                      
                      if (data.sam2_auto_run && data.sam2_result && data.sam2_result.overlay_base64) {{
                        console.log("✅ Updating viewer with overlay!");
                        const overlayUrl = "data:image/png;base64," + data.sam2_result.overlay_base64;
                        viewer.destroy();
                        pannellum.viewer("panoViewer", {{
                          type: "equirectangular",
                          panorama: overlayUrl,
                          autoLoad: true,
                          haov: 360,
                          vaov: 180,
                          hfov: 90,
                          showZoomCtrl: true,
                          compass: false
                        }});
                        
                        // Check if processing is complete (SAM2 + mask2cluster)
                        if (data.sam2_result.success && data.sam2_result.las_path) {{
                          console.log("✅✅ SAM2 + mask2cluster processing complete!");
                          console.log("LAS file:", data.sam2_result.las_path);
                          
                          // Show the "Show Results" button
                          document.getElementById('showResultsButton').style.display = 'block';
                          
                          // Notify the parent window (Streamlit) via a simple backend endpoint
                          // Store the completion status on the server side
                          fetch("{BRIDGE_SERVER_URL}/set_completion", {{
                            method: "POST",
                            headers: {{ "Content-Type": "application/json" }},
                            body: JSON.stringify({{
                              las_path: data.sam2_result.las_path,
                              completed: true,
                              overlay_base64: data.sam2_result.overlay_base64
                            }})
                          }})
                          .then(() => {{
                            console.log("✅ Notified backend of completion (with overlay saved)");
                          }})
                          .catch(err => console.error("Error notifying completion:", err));
                        }}
                      }} else {{
                        console.warn("❌ Not updating viewer - conditions not met");
                      }}
                    }})
                    .catch(err => {{
                      console.error("❌ Error sending click:", err);
                      // Always hide loading overlay on error
                      document.getElementById('loadingOverlay').style.display = 'none';
                      alert("Error processing click: " + err.message);
                    }});
                }}
              }}
            }});


          // Track view changes
          function sendView() {{
            const yaw = viewer.getYaw();
            const pitch = viewer.getPitch();
            const hfov = viewer.getHfov();
            window.parent.postMessage({{
              type: 'VIEW_UPDATE',
              yaw: yaw,
              pitch: pitch,
              hfov: hfov
            }}, '*');
          }}
          viewer.on('yawchange', sendView);
          viewer.on('pitchchange', sendView);
          viewer.on('zoomchange', sendView);

          document.getElementById('panoViewer').addEventListener('contextmenu', e => e.preventDefault());
        </script>
        <style>
          .custom-hotspot {{
            background: rgba(255,50,50,0.9);
            border: 2px solid white;
            border-radius: 50%;
            width: 18px; height: 18px;
          }}
          .custom-hotspot:hover {{
            background: rgba(255,100,100,1);
            transform: scale(1.2);
          }}
          
          /* Loading spinner animation */
          .spinner {{
            border: 4px solid rgba(255,255,255,0.3);
            border-top: 4px solid white;
            border-radius: 50%;
            width: 50px;
            height: 50px;
            animation: spin 1s linear infinite;
            margin: 0 auto;
          }}
          
          @keyframes spin {{
            0% {{ transform: rotate(0deg); }}
            100% {{ transform: rotate(360deg); }}
          }}
        </style>
        """
        components.html(html, height=height, scrolling=False)


    # Main UI
    n = len(pano_files)
    current_idx = st.session_state.pano_idx = wrap_idx(st.session_state.pano_idx, n)
    current_path = pano_files[current_idx]
    caption = os.path.basename(current_path)

    data_url = st.session_state.overlay_url or file_to_data_url(current_path)

    st.write(f"**Current panorama:** {caption} ({current_idx + 1}/{n})")

    cols = st.columns([1, 1, 2, 1, 1])
    with cols[0]:
        if st.button("◀ Previous", use_container_width=True):
            st.session_state.pano_idx = (current_idx - 1) % n
            st.session_state.overlay_url = None
            st.session_state.clicked_points = []
            st.session_state.viewer_key += 1
            st.rerun()
    with cols[1]:
        if st.button("Next ▶", use_container_width=True):
            st.session_state.pano_idx = (current_idx + 1) % n
            st.session_state.overlay_url = None
            st.session_state.clicked_points = []
            st.session_state.viewer_key += 1
            st.rerun()

    with cols[2]:
        button_label = (
            "Click mode" if not st.session_state.get("click_mode", False)
            else "Stop clicking mode"
        )

        if st.button(button_label, use_container_width=True):
            if st.session_state.get("click_mode", False):
                st.info("Stopping click mode... resetting server click data.")
                try:
                    res = requests.post(f"{BRIDGE_SERVER_URL}/click/reset", timeout=3)
                    if res.status_code == 200:
                        st.success("Click data cleared successfully.")
                    else:
                        st.warning(f"Reset returned {res.status_code}")
                except Exception as e:
                    st.error(f"Could not reset clicks: {e}")
                st.session_state.click_mode = False

                st.rerun()

            else:
                try:
                    requests.post(f"{BRIDGE_SERVER_URL}/click/reset", timeout=3)
                except Exception:
                    pass
                st.session_state.click_mode = True
                st.success("Click mode enabled — you can now select points.")

    render_pannellum_with_clicks(
        data_url,
        existing_points=st.session_state.clicked_points,
        image_filename=os.path.basename(current_path)
    )

    # Check if processing is complete by polling the backend
    if not st.session_state.processing_complete:
        try:
            response = requests.get(f"{BRIDGE_SERVER_URL}/get_completion", timeout=1)
            if response.status_code == 200:
                completion_data = response.json()
                if completion_data.get("completed"):
                    st.session_state.processing_complete = True
                    st.session_state.las_path = completion_data.get("las_path")
                    # Save the overlay to session state to persist it across reruns
                    overlay_base64 = completion_data.get("overlay_base64")
                    if overlay_base64:
                        st.session_state.overlay_url = f"data:image/png;base64,{overlay_base64}"
                    st.rerun()
        except:
            pass  # Ignore errors during polling
    
    # Display selected points
    if st.session_state.clicked_points:
        w, h = Image.open(current_path).size
        st.subheader("Selected Points")
        for i, p in enumerate(st.session_state.clicked_points, start=1):
            px, py = pitch_yaw_to_pixel(p["pitch"], p["yaw"], w, h)
            st.text(f"Point {i}: pitch={p['pitch']:.2f}°, yaw={p['yaw']:.2f}° → pixel=({px},{py})")
    else:
        st.caption("Click inside the panorama (max 5 points).")

    # Show "Show Results" button when processing is complete
    if st.session_state.processing_complete:
        st.success("✅ SAM2 and Mask2Cluster processing complete!")
        st.divider()
        
        col1, col2 = st.columns([1, 3])
        with col1:
            if st.button("🔍 Show Results", use_container_width=True, type="primary"):
                # Execute visualize_latest.py and get the URL
                try:
                    # Find script path: go up one level from GUI_SpatialLLM to SpatialLLM root
                    current_dir = os.path.dirname(os.path.abspath(__file__))
                    root_dir = os.path.dirname(current_dir)
                    script_path = os.path.join(root_dir, "SAM23D", "visualize_latest.py")
                    
                    if not os.path.exists(script_path):
                        st.error(f"Script not found: {script_path}")
                    else:
                        result = subprocess.run(
                            ["python3", script_path],
                            capture_output=True,
                            text=True,
                            timeout=30
                        )
                        
                        if result.returncode == 0:
                            # Extract URL from last line
                            url = result.stdout.strip().split('\n')[-1]
                            if url.startswith('http'):
                                # Open URL in new tab using streamlit_js_eval
                                streamlit_js_eval(js_expressions=f"window.open('{url}', '_blank')")
                            else:
                                st.error("Failed to get viewer URL")
                                st.code(result.stdout)
                        else:
                            st.error(f"Error running visualize_latest.py: {result.stderr}")
                except Exception as e:
                    st.error(f"Failed to launch viewer: {str(e)}")
        with col2:
            if st.session_state.las_path:
                st.caption(f"📁 Point cloud saved: `{os.path.basename(st.session_state.las_path)}`")

#  Chat UI
agent = st.session_state.get("agent")

if agent:
    st.divider()
    st.subheader("Spatial AI Chat")

    # Initialize chat history once
    if "chat_ui" not in st.session_state:
        st.session_state["chat_ui"] = []

    # Display existing chat
    for role, content in st.session_state["chat_ui"]:
        with st.chat_message(role):
            st.write(content)

    # Input box
    user_q = st.chat_input("Ask about the room… (type 'overview' or 'quit' for commands)")

    if user_q:
        with st.chat_message("user"):
            st.write(user_q)
        st.session_state["chat_ui"].append(("user", user_q))

        cleaned = user_q.strip().lower()

        if cleaned in {"quit", "exit", "q"}:
            with st.chat_message("assistant"):
                st.write("Goodbye! (You can reset the session or exit virtual mode.)")
            st.session_state["chat_ui"].append(
                ("assistant", "Goodbye! (You can reset the session or exit virtual mode.)")
            )

        elif cleaned == "overview":
            try:
                overview = agent.get_room_summary(agent.current_room_id)
            except Exception:
                # fallback if attribute not available
                overview = "Overview not available for this dataset."
            with st.chat_message("assistant"):
                st.markdown(f"<pre style='color:white;'>{overview}</pre>", unsafe_allow_html=True)
            st.session_state["chat_ui"].append(("assistant", overview))

        else:
            with st.spinner("Analyzing spatial data…"):
                try:
                    result = agent.query(user_q)
                except Exception as e:
                    result = {"error": str(e)}

            if "error" in result:
                with st.chat_message("assistant"):
                    st.error(result["error"])
                st.session_state["chat_ui"].append(("assistant", f"{result['error']}"))
            else:
                reply = result.get("response", "")
                used_images = result.get("used_images", False)
                with st.chat_message("assistant"):
                    st.write(reply if reply else "(no response)")
                    # if used_images:
                    # st.info("Image analysis was used for this response.")
                st.session_state["chat_ui"].append(("assistant", reply if reply else "(no response)"))