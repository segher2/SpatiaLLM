# SpatialLLM - Spatial Understanding with Language Models

[![Release](https://img.shields.io/github/v/release/Jackson513ye/LM2PCG?sort=semver)](https://github.com/Jackson513ye/LM2PCG/releases)

A comprehensive framework for spatial understanding combining point cloud processing, 3D segmentation, and natural language interfaces. This project integrates indoor scene reconstruction, interactive 3D visualization, and AI-powered spatial reasoning.

## 🌟 Overview

SpatialLLM consists of three integrated modules:

1. **[LM2PCG](./LM2PCG)** - Indoor Point Cloud Processing Pipeline
2. **[SAM23D](./SAM23D)** - 2D-to-3D Segmentation with SAM2
3. **[GUI_SpatialLLM](./GUI_SpatialLLM)** - Interactive Spatial Understanding GUI

## 📦 Modules

### 1. LM2PCG - Point Cloud Processing

A C++17 pipeline for indoor point-cloud processing with PCL and CGAL. Provides clustering, reconstruction, geometric analysis, and AI-powered spatial queries.

**Key Features:**
- 🎨 Color-preserving PLY I/O with full XYZRGB support
- 📦 FEC-style clustering with upright bounding boxes
- 🔄 Poisson mesh reconstruction with acceptance checks
- 📐 Accurate geometry analysis (volume, area, dominant color)
- 🤖 Unified AI API with `<OPERATION> <ID>` format
- 🌐 Interactive web viewer with real-time selection
- 📊 JSON-first output for AI agents

**Quick Start:**
```bash
cd LM2PCG
./pcg.sh "./data/rooms/Full House"  # Auto-build and process
```

[📖 Full LM2PCG Documentation](./LM2PCG/README.md)

### 2. SAM23D - 2D Segmentation to 3D Point Cloud

Pipeline for converting 2D image segmentation masks (from SAM2) to filtered 3D point clouds.

**Key Features:**
- 🖼️ SAM2.1 image segmentation on panoramic images
- 🔍 3D point cloud projection and filtering
- 📍 Camera pose-aware coordinate transformation
- 📤 LAS format output for filtered point clouds

**Components:**
- `sam2_predictor.py` - SAM2 segmentation
- `select_points_in_mask.py` - Point cloud filtering
- `conversion_2D_3D.py` - Coordinate transformations

**Quick Start:**
```bash
cd SAM23D
python sam2_predictor.py  # Run segmentation
```

[📖 Full SAM23D Documentation](./SAM23D/README.md)

### 3. GUI_SpatialLLM - Interactive Spatial Understanding

Web-based GUI for spatial understanding with panoramic images and 3D point clouds.

**Key Features:**
- 🌐 360° panorama viewer with interactive point selection
- 🎯 Real-time SAM2 segmentation overlay
- 💬 Natural language spatial queries via Azure OpenAI
- 🗄️ SQLite database for spatial data management
- 🔗 Flask + Streamlit architecture

**Quick Start:**
```bash
cd GUI_SpatialLLM
python bridge_server_final.py  # Terminal 1
streamlit run GUI_streamlit.py  # Terminal 2
```

[📖 Full GUI Documentation](./GUI_SpatialLLM/README.md)

## 🚀 Complete Pipeline Setup

### Requirements

**System:**
- Python 3.12+
- CMake 3.16+, C++17 compiler
- Bash shell (Linux, macOS, WSL, or Git Bash on Windows)

**Dependencies:**
- PCL 1.10+, CGAL 5.3+, Boost, Eigen3
- PyTorch, SAM2, Streamlit, Flask
- PolyFit wheel from [LiangliangNan/PolyFit](https://github.com/LiangliangNan/PolyFit/releases)
- Easy3D wheel from [LiangliangNan/Easy3D](https://github.com/LiangliangNan/Easy3D/releases)

### Installation

#### 1. Clone with Submodules

```bash
git clone --recurse-submodules https://github.com/segher2/SpatialLLM.git
cd SpatialLLM

# If already cloned without submodules:
git submodule update --init --recursive
```

#### 2. Set up Python Environment

```bash
python3.12 -m venv .venv
source .venv/bin/activate  # On Windows: .venv/Scripts/activate
pip install --upgrade pip wheel setuptools
pip install -r requirements.txt
pip install /path/to/PolyFit-<version>-cp312-*.whl
pip install /path/to/easy3D-<version>-cp312-*.whl
```

#### 3. Install System Dependencies

**macOS:**
```bash
brew install cmake cgal boost eigen pcl
```

**Ubuntu 22.04+:**
```bash
sudo apt-get install cmake build-essential libpcl-dev libcgal-dev libeigen3-dev libboost-all-dev
```

#### 4. Build C++ Components

```bash
cd LM2PCG
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j
cd ../..
```

#### 5. Download SAM2 Model

```bash
cd SAM23D/SAM2/checkpoints
# Download sam2.1_hiera_large.pt
cd ../../..
```

## 📊 Complete Workflow

### Option 1: Automated Pipeline (Recommended)

```bash
# 1. Place input data
mkdir -p data/input
cp /path/to/scan.e57 data/input/
cp /path/to/scan_segmented.las data/input/

# 2. Run Snakemake pipeline
snakemake --cores all --configfile config.yaml

# Output: data/output/_PCG_DONE marker indicates completion
```

### Option 2: Step-by-Step Manual Workflow

```bash
# 1. Process point clouds (LM2PCG)
cd LM2PCG
./pcg.sh "./data/rooms/Full House"

# 2. Start GUI for interactive segmentation
cd ../GUI_SpatialLLM
python bridge_server_final.py &  # Background
streamlit run GUI_streamlit.py

# 3. Select points in panorama → SAM2 segments → Filter 3D points
# 4. Query spatial relationships via AI API
cd ../LM2PCG
python scripts/ai_api.py VIS 0-7-12  # Visualize object
python scripts/ai_api.py VOL 0-7-12  # Get volume
python scripts/ai_api.py BBD 0-7-12 0-7-15  # Distance between objects
```

## 🤖 AI API Examples

```bash
cd LM2PCG

# Mesh reconstruction
python scripts/ai_api.py RCN 0-7-12

# Geometric analysis
python scripts/ai_api.py VOL 0-7-12     # Volume
python scripts/ai_api.py ARE 0-7-12     # Surface area
python scripts/ai_api.py CLR 0-7-12     # Dominant color

# Spatial relationships
python scripts/ai_api.py BBD 0-7-12 0-7-15  # Distance

# Interactive visualization
python scripts/ai_api.py VIS 0-7        # Room visualization
python scripts/ai_api.py VIS 0-7-12     # Object visualization

# Room summary
python scripts/ai_api.py RMS
```

## 📁 Project Structure

```
SpatialLLM/
├── LM2PCG/                  # Point cloud processing (submodule)
│   ├── build/               # C++ executables
│   ├── scripts/             # AI API scripts
│   ├── web/                 # 3D viewer
│   └── data/                # Room datasets
│
├── SAM23D/                  # 2D-3D segmentation (submodule)
│   ├── sam2_predictor.py
│   ├── select_points_in_mask.py
│   ├── conversion_2D_3D.py
│   └── SAM2/                # SAM2 model files
│
├── GUI_SpatialLLM/          # Interactive GUI (submodule)
│   ├── GUI_streamlit.py
│   ├── bridge_server_final.py
│   ├── extracted_data/      # Panoramas & metadata
│   └── outputs/             # Segmentation results
│
├── config.yaml              # Pipeline configuration
├── Snakefile               # Automated workflow
└── requirements.txt         # Python dependencies
```

## 🔧 Configuration

Edit `config.yaml` to customize:
- Input/output paths
- Processing parameters
- AI model settings
- Database connections

## 📝 Output Files

After running the pipeline, outputs are organized as:

```
data/output/
├── floor_0/
│   └── room_*/
│       ├── results/
│       │   ├── filtered_clusters/    # Segmented objects
│       │   ├── recon/                # Reconstructed meshes
│       │   └── shell/                # Room shells
│       └── *.csv                     # Object manifests
└── _PCG_DONE                         # Completion marker
```

## 🐛 Troubleshooting

### CGAL Version Issues
If CGAL < 5.3, mesh processing tools will be skipped. See [LM2PCG/docs/CGAL_VERSION_TROUBLESHOOTING.md](./LM2PCG/docs/CGAL_VERSION_TROUBLESHOOTING.md)

### Windows Issues
- Use Git Bash or WSL (PowerShell/CMD not supported)
- Ensure bash is available in PATH

### SAM2 Model Not Found
Download `sam2.1_hiera_large.pt` to `SAM23D/SAM2/checkpoints/`

## 📖 Documentation

- [LM2PCG Documentation](./LM2PCG/README.md) - Point cloud processing
- [SAM23D Documentation](./SAM23D/README.md) - 2D-3D segmentation
- [GUI Documentation](./GUI_SpatialLLM/README.md) - Interactive interface
- [AI API Guide](./LM2PCG/docs/AI_API.md) - API reference
- [Web Viewer Guide](./LM2PCG/docs/POINTCLOUD_VIEWER.md) - 3D visualization

## 🤝 Contributing

Each submodule has its own repository:
- [LM2PCG](https://github.com/Jackson513ye/LM2PCG)
- [SAM23D](https://github.com/Jackson513ye/SAM23D)
- [GUI_SpatialLLM](https://github.com/Jackson513ye/GUI_SpatialLLM)

Please submit issues and pull requests to the respective repositories.

## 📄 License

See individual submodule repositories for license information.

## ✅ Quick Checklist

- [ ] Python 3.12 venv active
- [ ] System dependencies installed (PCL, CGAL, Boost, Eigen)
- [ ] PolyFit + Easy3D wheels installed
- [ ] SAM2 model checkpoint downloaded
- [ ] Input `.e57` and `.las` files in `data/input/`
- [ ] Submodules initialized: `git submodule update --init --recursive`
- [ ] LM2PCG C++ components built
- [ ] Run: `snakemake --cores all --configfile config.yaml`

---

**Version:** 1.3.0  
**Last Updated:** 2025-11-03
