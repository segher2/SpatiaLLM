import subprocess
import json
import os
from typing import Dict, Any, List, Optional
from pydantic import BaseModel, Field, ValidationError


# --- Pydantic Schemas ---

class VolOutput(BaseModel):
    mesh: str
    volume: float
    closed: bool


class ClrOutput(BaseModel):
    object_code: str
    M: int
    components: List[Dict[str, Any]]


class BbdOutput(BaseModel):
    distance: float
    vector_1_to_2: Dict[str, float]
    center1: Optional[Dict[str, float]] = None
    center2: Optional[Dict[str, float]] = None


# NEW: Output schema for VIS tool
class VisOutput(BaseModel):
    status: str
    mode: Optional[str] = None
    name: Optional[str] = None
    viewer_url: Optional[str] = None
    objects: Optional[List[str]] = None
    requires_selection: bool = False  # NEW: Indicates if user selection is needed
    session_id: Optional[str] = None  # NEW: Selection session identifier
    selection_file: Optional[str] = None  # NEW: Path to selection result file
    error: Optional[str] = None


# ------------------------------

class AiApiWrapper:
    """
    Handles communication with the external C++/Python pipeline API
    by executing commands and robustly parsing structured output.
    Uses simplified <OPERATION> <ID> format.
    """

    def __init__(self, api_script_path: str = "scripts/ai_api.py"):
        self.api_script_path = api_script_path
        self.is_ready = self._check_api_readiness()

    def _check_api_readiness(self) -> bool:
        """Verifies the Python API script exists and can run."""
        if not os.path.exists(self.api_script_path):
            print(f"❌ API Error: ai_api.py script not found at {self.api_script_path}")
            return False
        # Add basic python3 check if needed
        try:
            subprocess.run(["python3", "--version"], capture_output=True, check=True)
        except (FileNotFoundError, subprocess.CalledProcessError):
            print("❌ API Error: python3 interpreter not found or not working.")
            return False
        print(f"✅ API Wrapper initialized, targeting: {self.api_script_path}")
        return True

    def _execute_command(self, head_code: str, *args: str) -> Optional[List[Dict[str, Any]]]:
        """ Executes external API, handles timeout, returns parsed JSON results. """
        if not self.is_ready: return None

        command = ["python3", self.api_script_path, head_code]
        command.extend(args)
        # REMOVED: No longer adding "--json" flag

        try:
            print(f"⚙️ Executing API: {' '.join(command)}")
            # MODIFIED: Increased timeout to 240 seconds
            result = subprocess.run(
                command, capture_output=True, text=True, check=True,
                encoding='utf-8', timeout=240
            )

            json_results = []
            for line in result.stdout.strip().splitlines():
                if line.strip():
                    try:
                        json_results.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue  # Skip non-JSON lines

            # Handle case where C++ tool might print JSON but exit with 0
            if not json_results and result.stdout.strip():
                print(
                    f"⚠️ API Info ({head_code}): Command succeeded but produced non-JSON output:\n{result.stdout.strip()}")
                # Decide if this should be an error or just ignored based on tool behavior
                # For now, treat as no result if no valid JSON found
                return None

            # Return empty list if tool produced no output (e.g., VIS in no-wait?)
            # Or handle specific tool outputs if needed
            return json_results if json_results else None  # Return None if truly empty

        except subprocess.TimeoutExpired:
            print(f"❌ API Call Failed ({head_code}): Command timed out after 240 seconds.")  # Updated message
            return None
        except subprocess.CalledProcessError as e:
            error_output = e.stderr.strip() if e.stderr else e.stdout.strip()
            # Try to parse error output as JSON for more structured errors
            error_json = None
            if error_output.startswith('{') and error_output.endswith('}'):
                try:
                    error_json = json.loads(error_output)
                except json.JSONDecodeError:
                    pass

            if error_json and 'error_message' in error_json:
                error_msg = error_json['error_message']
            elif error_output:
                error_msg = error_output
            else:
                error_msg = f"C++ tool crashed unexpectedly (Exit Code {e.returncode})."

            print(f"❌ API Call Failed ({head_code}): Command failed.")
            print(f"   Exit Code: {e.returncode}")
            print(f"   Error: {error_msg}")
            return None
        except FileNotFoundError:
            print(f"❌ API Execution Failed: python3 or {self.api_script_path} not found.")
            return None
        except Exception as e:  # Catch other potential errors
            print(f"❌ Unexpected API Execution Error ({head_code}): {e}")
            traceback.print_exc()
            return None

    # --- Tool Functions ---

    def calculate_volume(self, object_code: str) -> Optional[VolOutput]:
        """Calculates volume. Command: VOL <object_code>"""
        # REMOVED: "--object" flag
        results = self._execute_command("VOL", object_code)
        if results and results[0]:
            try:
                # Basic check for expected keys before parsing
                if 'volume' in results[0] and 'mesh' in results[0]:
                    return VolOutput(**results[0])
                print(f"❌ Volume Parsing Error: Missing expected keys in result for {object_code}.")
                return None
            except ValidationError as e:
                print(f"❌ Volume Parsing Validation Error for {object_code}: {e}")
                return None
            except Exception as e:
                print(f"❌ Volume General Parsing Error for {object_code}: {e}")
                return None
        return None

    def analyze_dominant_color(self, object_code: str) -> Optional[ClrOutput]:
        """Analyzes color. Command: CLR <object_code>"""
        # REMOVED: "--object" flag
        results = self._execute_command("CLR", object_code)
        if results and results[0]:
            try:
                # Check based on expected successful output structure
                if 'M' in results[0] and 'components' in results[0]:
                    return ClrOutput(object_code=object_code, **results[0])
                # Check if it's a known error structure from the tool
                elif 'error_message' in results[0]:
                    print(f"❌ CLR Tool Error reported: {results[0]['error_message']}")
                    return None
                else:
                    print(f"❌ Color Parsing Error: Unexpected result structure for {object_code}.")
                    return None
            except ValidationError as e:
                print(f"❌ Color Parsing Validation Error for {object_code}: {e}")
                return None
            except Exception as e:
                print(f"❌ Color General Parsing Error for {object_code}: {e}")
                return None
        return None

    def calculate_bbox_distance(self, object_code_1: str, object_code_2: str) -> Optional[BbdOutput]:
        """Calculates distance. Command: BBD <code1> <code2>"""
        results = self._execute_command("BBD", object_code_1, object_code_2)
        if results and results[0]:
            try:
                # Ensure distance key exists at minimum
                if 'distance' in results[0]:
                    return BbdOutput(**results[0])
                elif 'error_message' in results[0]:
                    print(f"❌ BBD Tool Error reported: {results[0]['error_message']}")
                    return None
                else:
                    print(f"❌ BBD Parsing Error: Unexpected result structure for {object_code_1}/{object_code_2}.")
                    return None
            except ValidationError as e:
                print(f"❌ BBD Parsing Validation Error for {object_code_1}/{object_code_2}: {e}")
                return None
            except Exception as e:
                print(f"❌ BBD General Parsing Error for {object_code_1}/{object_code_2}: {e}")
                return None
        return None

    # UPDATED: Simplified visualize function
    def visualize_point_cloud(self, codes: List[str]) -> Optional[VisOutput]:
        """ Launches visualization. Command: VIS <code1> [code2 ...] """
        if not codes:
            print("❌ VIS Error: No codes provided for visualization.")
            return None

        # Pass codes directly as arguments
        results = self._execute_command("VIS", *codes)

        if results and results[0]:
            try:
                # Basic check for status key
                if 'status' in results[0]:
                    return VisOutput(**results[0])
                elif 'error_message' in results[0]:
                    # Handle potential error JSON from the tool itself
                    print(f"❌ VIS Tool Error reported: {results[0]['error_message']}")
                    # Return an error VisOutput object
                    return VisOutput(status="error", error=results[0]['error_message'], objects=codes)
                else:
                    print(f"❌ VIS Parsing Error: Unexpected result structure for codes: {codes}.")
                    return VisOutput(status="error", error="Unexpected result structure", objects=codes)
            except ValidationError as e:
                print(f"❌ VIS Parsing Validation Error for codes {codes}: {e}")
                return VisOutput(status="error", error=f"Validation Error: {e}", objects=codes)
            except Exception as e:
                print(f"❌ VIS General Parsing Error for codes {codes}: {e}")
                return VisOutput(status="error", error=f"General Error: {e}", objects=codes)
        else:
            # If _execute_command returned None (due to timeout, crash, etc.)
            print(f"❌ VIS tool execution returned no results for codes: {codes}")
            return VisOutput(status="error", error="Tool execution failed or produced no output.", objects=codes)

