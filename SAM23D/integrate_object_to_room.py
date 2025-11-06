#!/usr/bin/env python3
"""
Object Integration Pipeline for SAM23D

This script integrates user-selected objects into the room structure:
1. Finds the latest *_cropped.png file in SAM23D/outputs
2. Sends it to Azure OpenAI GPT-4 Vision for semantic labeling
3. Gets semantic labels (single-word object descriptions)
4. Finds the corresponding room based on panorama filename
5. Copies *_test.ply to room's filtered_clusters/userselected_XXX/
6. Computes UOBB (Upright Oriented Bounding Box)
7. Updates room CSV with new object entry

Usage:
    python integrate_object_to_room.py
"""

import os
import json
import base64
import csv
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, List, Tuple
from openai import AzureOpenAI
from dotenv import load_dotenv


class ObjectIntegrator:
    """Handles the complete pipeline: semantic labeling → room integration → UOBB computation."""
    
    def __init__(self):
        """Initialize the labeler with Azure OpenAI client."""
        self.base_dir = Path(__file__).resolve().parent
        self.outputs_dir = self.base_dir / "outputs"
        
        # Load API key from LM2PCG/data/configs/.env
        env_path = self.base_dir.parent / "LM2PCG" / "data" / "configs" / ".env"
        if env_path.exists():
            load_dotenv(env_path)
            print(f"✅ Loaded .env from: {env_path}")
        else:
            print(f"⚠️  .env file not found at: {env_path}")
        
        # Initialize Azure OpenAI client
        api_key = os.getenv("API_KEY")
        if not api_key:
            raise ValueError("❌ API_KEY not found in environment variables")
        
        self.client = AzureOpenAI(
            azure_endpoint="https://azure-openai-scanplan.openai.azure.com/",
            api_key=api_key,
            api_version="2025-02-01-preview"
        )
        print("✅ Azure OpenAI client initialized")
    
    def find_latest_cropped_image(self) -> Optional[Path]:
        """Find the most recently created *_cropped.png file in outputs directory."""
        if not self.outputs_dir.exists():
            print(f"❌ Outputs directory not found: {self.outputs_dir}")
            return None
        
        # Find all cropped images
        cropped_files = list(self.outputs_dir.rglob("*_cropped.png"))
        
        if not cropped_files:
            print(f"❌ No *_cropped.png files found in {self.outputs_dir}")
            return None
        
        # Sort by modification time, newest first
        latest = max(cropped_files, key=lambda p: p.stat().st_mtime)
        print(f"📁 Found latest cropped image: {latest.name}")
        
        return latest
    
    def encode_image_to_base64(self, image_path: Path) -> str:
        """Encode image to base64 string."""
        with open(image_path, "rb") as f:
            image_data = f.read()
        return base64.b64encode(image_data).decode("utf-8")
    
    def get_semantic_labels(self, image_path: Path) -> Dict:
        """
        Send image to GPT-4 Vision and get semantic labels.
        
        Returns:
            Dict with labels, confidence, and raw response
        """
        print(f"🔄 Encoding image: {image_path.name}")
        image_base64 = self.encode_image_to_base64(image_path)
        
        # Construct prompt for GPT-4 Vision
        prompt = """Identify the MAIN object in this cropped panoramic image.
Return ONLY ONE single-word label in English.
Focus on: furniture, architectural elements, appliances, fixtures.
Examples: sofa, window, door, table, chair, tv, lamp, bed, cabinet
Your response (one word only):"""
        
        print("🤖 Calling Azure OpenAI GPT-4 Vision...")
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",  # Using same model as other parts of the project
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": prompt
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/png;base64,{image_base64}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=150,
                temperature=0.0
            )
            
            # Extract response
            raw_response = response.choices[0].message.content.strip()
            print(f"✅ GPT-4 Vision response: {raw_response}")
            
            # Parse label (single word only, take first word if multiple)
            label = raw_response.split(",")[0].split()[0].strip().lower()
            
            result = {
                "image_path": str(image_path),
                "timestamp": datetime.now().isoformat(),
                "label": label,  # Changed from "labels" to "label" (singular)
                "raw_response": raw_response,
                "model": "gpt-4o-mini",
                "success": True
            }
            
            return result
            
        except Exception as e:
            print(f"❌ Error calling GPT-4 Vision: {e}")
            import traceback
            traceback.print_exc()
            
            return {
                "image_path": str(image_path),
                "timestamp": datetime.now().isoformat(),
                "label": None,  # Changed from "labels" to "label"
                "raw_response": None,
                "error": str(e),
                "success": False
            }
    
    def save_result(self, result: Dict, image_path: Path):
        """Save labeling result as JSON file."""
        # Save next to the cropped image
        output_json = image_path.with_suffix(".json").with_name(
            image_path.stem + "_labels.json"
        )
        
        with open(output_json, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        
        print(f"💾 Saved labels to: {output_json}")
        return output_json
    
    def run(self):
        """Main execution: find image, label it, save results."""
        print("=" * 60)
        print("🏷️  Semantic Labeler for Cropped Panoramas")
        print("=" * 60)
        
        # 1. Find latest cropped image
        image_path = self.find_latest_cropped_image()
        if not image_path:
            print("❌ No image to process. Exiting.")
            return False
        
        # 2. Get semantic labels from GPT-4 Vision
        result = self.get_semantic_labels(image_path)
        
        # 3. Save results
        if result.get("success"):
            output_json = self.save_result(result, image_path)
            
            print("\n" + "=" * 60)
            print("✅ Labeling Complete!")
            print("=" * 60)
            print(f"Image:  {image_path.name}")
            print(f"Label:  {result['label']}")  # Changed from "Labels" to "Label"
            print(f"Output: {output_json.name}")
            print("=" * 60)
            
            # 4. Integrate into room structure
            print("\n" + "=" * 60)
            print("🔄 Integrating into Room Structure")
            print("=" * 60)
            
            image_stem = image_path.stem.replace("_cropped", "")
            semantic_label = result['label']
            
            success_integrate = self.integrate_to_room(image_stem, semantic_label)
            
            if success_integrate:
                print("\n✅ Complete! Labeling and Integration Done!")
            else:
                print("\n⚠️  Labeling succeeded but integration failed.")
            
            return True
        else:
            print("\n❌ Labeling failed. Check error messages above.")
            return False
    
    def find_room_for_panorama(self, image_stem: str) -> Optional[Tuple[Path, int, int]]:
        """
        Find the room folder containing the panorama with the given stem.
        Returns (room_path, floor_number, room_number) or None.
        """
        # Search in data/output structure
        base_output = self.base_dir.parent / "data" / "output"
        
        if not base_output.exists():
            print(f"❌ Output directory not found: {base_output}")
            return None
        
        # Search all floors and rooms
        for floor_dir in sorted(base_output.glob("floor_*")):
            if not floor_dir.is_dir():
                continue
                
            # Extract floor number
            try:
                floor_num = int(floor_dir.name.split("_")[1])
            except (IndexError, ValueError):
                continue
            
            for room_dir in sorted(floor_dir.glob("room_*")):
                if not room_dir.is_dir():
                    continue
                
                # Extract room number
                try:
                    room_num = int(room_dir.name.split("_")[1])
                except (IndexError, ValueError):
                    continue
                
                # Check if panorama exists in this room
                for ext in ['.jpg', '.jpeg', '.png', '.tif', '.tiff']:
                    pano_file = room_dir / f"{image_stem}{ext}"
                    if pano_file.exists():
                        print(f"✅ Found panorama in: {room_dir}")
                        print(f"   Floor: {floor_num}, Room: {room_num}")
                        return room_dir, floor_num, room_num
        
        print(f"❌ No room found containing panorama: {image_stem}")
        return None
    
    def get_max_object_id_from_csv(self, csv_path: Path) -> int:
        """Read CSV and return the maximum object_id, or -1 if empty."""
        if not csv_path.exists():
            return -1
        
        max_id = -1
        try:
            with open(csv_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        obj_id = int(row['object_id'])
                        if obj_id > max_id:
                            max_id = obj_id
                    except (KeyError, ValueError):
                        continue
        except Exception as e:
            print(f"⚠️  Error reading CSV: {e}")
        
        return max_id
    
    def compute_uobb(self, cluster_ply: Path, uobb_ply: Path) -> Optional[Dict]:
        """
        Call pcg_bbox_single to compute UOBB.
        Returns dict with geometry parameters or None on failure.
        """
        # Find pcg_bbox_single executable
        bbox_exe = self.base_dir.parent / "LM2PCG" / "build" / "pcg_bbox_single"
        
        if not bbox_exe.exists():
            print(f"❌ pcg_bbox_single not found at: {bbox_exe}")
            print("   Please build it first: cd LM2PCG/build && cmake .. && make pcg_bbox_single")
            return None
        
        try:
            print(f"🔧 Computing UOBB...")
            print(f"   Input: {cluster_ply.name}")
            
            result = subprocess.run(
                [str(bbox_exe), str(cluster_ply), str(uobb_ply)],
                capture_output=True,
                text=True,
                check=True
            )
            
            # Parse JSON output from stdout
            output_lines = result.stdout.strip().split('\n')
            json_str = '\n'.join(output_lines)
            
            geometry = json.loads(json_str)
            print(f"✅ UOBB computed successfully")
            print(f"   Center: ({geometry['center_x']:.3f}, {geometry['center_y']:.3f}, {geometry['center_z']:.3f})")
            print(f"   Size: ({geometry['size_x']:.3f}, {geometry['size_y']:.3f}, {geometry['size_z']:.3f})")
            
            return geometry
            
        except subprocess.CalledProcessError as e:
            print(f"❌ UOBB computation failed: {e}")
            if e.stderr:
                print(f"   Error: {e.stderr}")
            return None
        except json.JSONDecodeError as e:
            print(f"❌ Failed to parse UOBB JSON output: {e}")
            return None
        except Exception as e:
            print(f"❌ UOBB computation error: {e}")
            return None
    
    def append_to_csv(self, csv_path: Path, row_data: Dict):
        """Append a new row to the room CSV file."""
        # Check if CSV exists
        file_exists = csv_path.exists()
        
        fieldnames = [
            'object_code', 'object_id', 'room_id', 'floor_id', 'class', 'file',
            'cluster_id', 'center_x', 'center_y', 'center_z',
            'size_x', 'size_y', 'size_z', 'yaw_rad'
        ]
        
        try:
            with open(csv_path, 'a', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                
                # Write header if file is new
                if not file_exists or csv_path.stat().st_size == 0:
                    writer.writeheader()
                
                writer.writerow(row_data)
            
            print(f"✅ Updated CSV: {csv_path.name}")
            
        except Exception as e:
            print(f"❌ Failed to update CSV: {e}")
            raise
    
    def integrate_to_room(self, image_stem: str, semantic_label: str) -> bool:
        """
        Main integration function:
        1. Find room folder
        2. Copy _test.ply to room/results/filtered_clusters/userselected_xxx/
        3. Compute UOBB
        4. Update CSV
        """
        try:
            # 1. Find room
            room_info = self.find_room_for_panorama(image_stem)
            if not room_info:
                return False
            
            room_dir, floor_num, room_num = room_info
            
            # 2. Find _test.ply
            test_ply_dir = self.outputs_dir / "filtered_point_clouds" / image_stem
            test_ply = test_ply_dir / f"{image_stem}_test.ply"
            
            if not test_ply.exists():
                print(f"❌ _test.ply not found: {test_ply}")
                return False
            
            print(f"✅ Found _test.ply: {test_ply.name}")
            
            # 3. Get next object_id
            csv_path = room_dir / f"room_{room_num:03d}.csv"
            max_obj_id = self.get_max_object_id_from_csv(csv_path)
            new_obj_id = max_obj_id + 1
            
            print(f"📊 Next object_id: {new_obj_id}")
            
            # 4. Create target directory
            target_dir = room_dir / "results" / "filtered_clusters" / f"userselected_{room_num:03d}"
            target_dir.mkdir(parents=True, exist_ok=True)
            print(f"📁 Target directory: {target_dir}")
            
            # 5. Generate filenames
            object_code = f"{floor_num}-{room_num}-{new_obj_id}"
            cluster_filename = f"{object_code}_{semantic_label}_cluster.ply"
            uobb_filename = f"{object_code}_{semantic_label}_uobb.ply"
            
            cluster_ply = target_dir / cluster_filename
            uobb_ply = target_dir / uobb_filename
            
            # 6. Copy _test.ply to target
            print(f"📋 Copying cluster PLY...")
            shutil.copy2(test_ply, cluster_ply)
            print(f"✅ Copied to: {cluster_ply.name}")
            
            # 7. Compute UOBB
            geometry = self.compute_uobb(cluster_ply, uobb_ply)
            if not geometry:
                print("❌ UOBB computation failed, but cluster file was copied.")
                return False
            
            # 8. Prepare CSV row
            csv_row = {
                'object_code': object_code,
                'object_id': new_obj_id,
                'room_id': room_num,
                'floor_id': floor_num,
                'class': semantic_label,
                'file': cluster_filename,
                'cluster_id': 0,  # Set to 0 as instructed
                'center_x': geometry['center_x'],
                'center_y': geometry['center_y'],
                'center_z': geometry['center_z'],
                'size_x': geometry['size_x'],
                'size_y': geometry['size_y'],
                'size_z': geometry['size_z'],
                'yaw_rad': geometry['yaw_rad']
            }
            
            # 9. Append to CSV
            self.append_to_csv(csv_path, csv_row)
            
            # 10. Summary
            print("\n" + "=" * 60)
            print("✅ Integration Summary")
            print("=" * 60)
            print(f"Object Code:   {object_code}")
            print(f"Class:         {semantic_label}")
            print(f"Cluster PLY:   {cluster_filename}")
            print(f"UOBB PLY:      {uobb_filename}")
            print(f"CSV Updated:   {csv_path.name}")
            print("=" * 60)
            
            # Output structured data for API parsing
            print("\n--- STRUCTURED_OUTPUT_START ---")
            print(f"object_code: {object_code}")
            print(f"room: floor_{floor_num}/room_{room_num:03d}")
            print(f"semantic_label: {semantic_label}")
            print(f"cluster_file: {cluster_filename}")
            print(f"uobb_file: {uobb_filename}")
            print("--- STRUCTURED_OUTPUT_END ---")
            
            return True
            
        except Exception as e:
            print(f"❌ Integration failed: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Entry point for the object integration pipeline."""
    try:
        integrator = ObjectIntegrator()
        success = integrator.run()
        return 0 if success else 1
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    import sys
    sys.exit(main())
