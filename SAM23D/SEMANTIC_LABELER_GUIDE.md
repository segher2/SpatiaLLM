# Semantic Labeler with Room Integration

## 概述

`semantic_labeler.py` 实现了从语义标注到房间数据集成的完整自动化流程。

## 完整工作流程

### 1. 语义标注
- 找到最新的 `*_cropped.png` 文件
- 调用 Azure OpenAI GPT-4o-mini 进行物体识别
- 生成单个英文单词标签（如 "door", "window", "table"）

### 2. 定位房间
- 使用逻辑 1（全景图暴力搜索）找到对应的房间文件夹
- 提取 floor_number 和 room_number

### 3. 文件集成
- 从 `SAM23D/outputs/filtered_point_clouds/{stem}/{stem}_test.ply` 复制点云
- 保存到 `data/output/floor_{X}/room_{YYY}/results/filtered_clusters/userselected_{YYY}/`
- 文件命名：`{floor}-{room}-{object_id}_{class}_cluster.ply`

### 4. UOBB 计算
- 调用 `LM2PCG/build/pcg_bbox_single` 可执行文件
- 计算上直立定向边界框（Upright Oriented Bounding Box）
- 生成 `*_uobb.ply` 文件
- 提取几何参数：center (x,y,z), size (x,y,z), yaw_rad

### 5. CSV 更新
- 读取 `room_{YYY}.csv`
- 找到最大 object_id
- 添加新行，字段包括：
  - object_code: `{floor_id}-{room_id}-{object_id}`
  - object_id: max_object_id + 1
  - room_id, floor_id
  - class: 语义标签
  - file: cluster PLY 文件名
  - cluster_id: 0
  - center_x, center_y, center_z
  - size_x, size_y, size_z
  - yaw_rad

## 使用方法

```bash
cd /Users/jacksonye/SpatiaLLM-final/SAM23D
python3 semantic_labeler.py
```

## 输出示例

### 终端输出
```
============================================================
🏷️  Semantic Labeler for Cropped Panoramas
============================================================
📁 Found latest cropped image: 00028_..._cropped.png
🤖 Calling Azure OpenAI GPT-4 Vision...
✅ GPT-4 Vision response: door

============================================================
🔄 Integrating into Room Structure
============================================================
✅ Found panorama in: data/output/floor_0/room_002
📊 Next object_id: 14
✅ Copied to: 0-2-14_door_cluster.ply
🔧 Computing UOBB...
✅ UOBB computed successfully
✅ Updated CSV: room_002.csv

============================================================
✅ Integration Summary
============================================================
Object Code:   0-2-14
Class:         door
Cluster PLY:   0-2-14_door_cluster.ply
UOBB PLY:      0-2-14_door_uobb.ply
CSV Updated:   room_002.csv
============================================================
```

### 生成的文件

```
data/output/floor_0/room_002/
├── room_002.csv                        # 更新的 CSV
└── results/
    └── filtered_clusters/
        └── userselected_002/
            ├── 0-2-14_door_cluster.ply  # 复制的点云
            └── 0-2-14_door_uobb.ply     # UOBB 边界框

SAM23D/outputs/
└── 00028_..._cropped_labels.json      # 标签 JSON
```

### CSV 新增行示例

```csv
object_code,object_id,room_id,floor_id,class,file,cluster_id,center_x,center_y,center_z,size_x,size_y,size_z,yaw_rad
0-2-14,14,2,0,door,0-2-14_door_cluster.ply,0,9.1616,10.5249,0.917777,1.15472,0.614599,1.5238,-0.0800158
```

## 技术组件

### C++ 可执行文件：`pcg_bbox_single`
- 位置：`LM2PCG/build/pcg_bbox_single`
- 源码：`LM2PCG/src/apps/pcg_bbox_single.cpp`
- 功能：为单个点云计算 UOBB
- 输出：UOBB PLY 文件 + JSON 格式的几何参数

### Python 脚本：`semantic_labeler.py`
- 位置：`SAM23D/semantic_labeler.py`
- 依赖：
  - `openai` - Azure OpenAI API
  - `python-dotenv` - 环境变量加载
  - `subprocess` - 调用 C++ 工具

## 配置

### API Key
位置：`LM2PCG/data/configs/.env`
```
API_KEY=<your_azure_openai_key>
```

### Azure OpenAI 配置
- Endpoint: `https://azure-openai-scanplan.openai.azure.com/`
- Model: `gpt-4o-mini`
- Temperature: 0.0
- Max tokens: 150

## 错误处理

### 常见问题

1. **pcg_bbox_single 不存在**
   ```bash
   cd LM2PCG/build
   cmake ..
   make pcg_bbox_single
   ```

2. **找不到房间**
   - 检查全景图是否在 `data/output/floor_*/room_*/` 中
   - 确认文件名 stem 完全匹配

3. **CSV 格式错误**
   - 确保 CSV 文件存在且格式正确
   - 检查 `object_id` 列是否包含有效整数

## 数据流图

```
User clicks 5 points
    ↓
SAM2 generates mask
    ↓
*_cropped.png saved
    ↓
semantic_labeler.py runs
    ↓
GPT-4 Vision → "door"
    ↓
Find room folder (逻辑 1)
    ↓
Copy *_test.ply → userselected_002/0-2-14_door_cluster.ply
    ↓
Compute UOBB → 0-2-14_door_uobb.ply
    ↓
Update room_002.csv
    ↓
✅ Complete!
```

## 未来改进

- [ ] 支持批量处理多个 cropped 图片
- [ ] 添加标签验证和校正机制
- [ ] 支持自定义标签词表
- [ ] 集成到 GUI 工作流中（自动触发）
- [ ] 添加撤销功能（删除错误添加的对象）

---

**最后更新**: 2025年11月6日
**作者**: Copilot + Jackson Ye
