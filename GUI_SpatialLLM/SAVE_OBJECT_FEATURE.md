# "Save New Object" 功能说明

## 概述

在 GUI 中添加了 **"💾 Save New Object"** 按钮，与 "🔍 Show Results" 按钮并排显示，用于将用户选择的对象自动集成到房间结构中。

## 功能位置

### 前端按钮 (JavaScript)
**文件**: `GUI_SpatialLLM/GUI_streamlit.py` (第 301-310 行)

```html
<div id="actionButtons" style="display:none;...">
  <button onclick="saveNewObject()">💾 Save New Object</button>
  <button onclick="showResults()">🔍 Show Results</button>
</div>
```

### 后端端点 (Flask)
**文件**: `GUI_SpatialLLM/bridge_server_final.py`

新增端点：`POST /save_object`

### 核心脚本（重命名）
**原文件名**: `SAM23D/semantic_labeler.py`  
**新文件名**: `SAM23D/integrate_object_to_room.py`  
**原因**: 新名称更准确地描述其功能（不仅仅是语义标注，还包括完整的房间集成流程）

---

## 显示时机

与 "Show Results" 按钮**完全相同**：

### 触发条件
1. 用户在全景图上点击了 **5 个点**
2. SAM2 生成 mask/overlay/cropped 图像成功
3. `mask2cluster_cli` 生成 `*_test.ply` 成功
4. Backend 返回 `sam2_result.success === true` 且包含 `las_path`

### 实现方式
```javascript
// 当处理完成时显示按钮容器
document.getElementById('actionButtons').style.display = 'block';
```

---

## 点击行为

### 前端 JavaScript (第 308-340 行)

```javascript
function saveNewObject() {
  // 1. 显示加载状态
  btn.innerHTML = '⏳ Processing...';
  btn.disabled = true;
  
  // 2. 调用 Bridge Server
  fetch("/save_object", {method: "POST"})
  
  // 3. 显示结果弹窗
  if (success) {
    alert("✅ Object saved successfully!\n" +
          "Room: floor_0/room_002\n" +
          "Object: 0-2-14 (door)\n" +
          "Files: 0-2-14_door_cluster.ply");
  }
}
```

### 后端 Bridge Server (第 120-175 行)

```python
@app.route('/save_object', methods=['POST'])
def save_object():
    # 1. 定位脚本
    script_path = "SAM23D/integrate_object_to_room.py"
    
    # 2. 执行脚本 (60秒超时)
    result = subprocess.run(["python3", script_path], 
                          capture_output=True, timeout=60)
    
    # 3. 解析输出
    # 从 STRUCTURED_OUTPUT 部分提取字段
    
    # 4. 返回 JSON
    return jsonify({
        "success": True,
        "object_code": "0-2-14",
        "room": "floor_0/room_002",
        "semantic_label": "door",
        "cluster_file": "0-2-14_door_cluster.ply"
    })
```

---

## 核心脚本：`integrate_object_to_room.py`

### 重命名原因
| 旧名称 | 新名称 | 原因 |
|--------|--------|------|
| `semantic_labeler.py` | `integrate_object_to_room.py` | 功能不仅限于语义标注，包括完整的房间集成流程 |
| `SemanticLabeler` 类 | `ObjectIntegrator` 类 | 更准确地反映职责 |

### 完整执行流程

```
1. find_latest_cropped_image()
   └─ 找到 SAM23D/outputs/*_cropped.png
   
2. get_semantic_labels()
   └─ 调用 Azure OpenAI GPT-4 Vision API
   └─ 获取单词标签（如 "door", "window"）
   
3. save_result()
   └─ 保存 JSON 结果到 SAM23D/outputs/*.json
   
4. integrate_to_room()
   ├─ find_room_for_panorama()
   │  └─ 根据全景图文件名找到对应房间
   │
   ├─ get_max_object_id_from_csv()
   │  └─ 读取 CSV，确定新对象 ID
   │
   ├─ 复制文件
   │  └─ *_test.ply → userselected_XXX/{floor}-{room}-{id}_{label}_cluster.ply
   │
   ├─ compute_uobb()
   │  └─ 调用 C++ 工具 pcg_bbox_single
   │  └─ 生成 *_uobb.ply
   │
   └─ append_to_csv()
      └─ 添加新行到 room_XXX.csv
```

### 结构化输出（新增）

为了让 Bridge Server 可以解析结果，脚本现在输出：

```
--- STRUCTURED_OUTPUT_START ---
object_code: 0-2-14
room: floor_0/room_002
semantic_label: door
cluster_file: 0-2-14_door_cluster.ply
uobb_file: 0-2-14_door_uobb.ply
--- STRUCTURED_OUTPUT_END ---
```

Bridge Server 解析这些行并返回给前端。

---

## 文件结构

### 输入文件
```
SAM23D/outputs/
├── R0002_link_4_rgb_0000_cropped.png    # 裁剪的全景图
└── filtered_point_clouds/
    └── R0002_link_4_rgb_0000/
        └── R0002_link_4_rgb_0000_test.ply  # FEC 聚类结果
```

### 输出文件
```
data/output/floor_0/room_002/
├── room_002.csv                         # ✅ 新增一行
└── results/filtered_clusters/
    └── userselected_002/
        ├── 0-2-14_door_cluster.ply     # ✅ 复制的点云
        └── 0-2-14_door_uobb.ply        # ✅ UOBB 边界框
```

### CSV 新增行示例
```csv
object_code,object_id,room_id,floor_id,class,file,cluster_id,center_x,center_y,center_z,size_x,size_y,size_z,yaw_rad
0-2-14,14,2,0,door,0-2-14_door_cluster.ply,0,9.1616,10.5249,0.917777,1.15472,0.614599,1.5238,-0.0800158
```

---

## UI/UX 细节

### 按钮样式
- **颜色**: 橙色 (`#FF9800`) - 表示保存操作
- **位置**: 屏幕底部居中，Show Results 按钮左侧
- **间距**: 10px 右边距

### 加载状态
```javascript
// 点击前
💾 Save New Object

// 点击后（处理中）
⏳ Processing...
[按钮禁用]

// 完成后恢复
💾 Save New Object
```

### 成功弹窗
```
✅ Object saved successfully!

Room: floor_0/room_002
Object: 0-2-14 (door)
Files: 0-2-14_door_cluster.ply
```

### 错误处理
```
❌ Error: Integration failed
[详细错误信息]
```

---

## 依赖关系

### Python 依赖
- `openai` - Azure OpenAI API 客户端
- `python-dotenv` - 加载 .env 文件
- `subprocess` - 调用 C++ 工具
- `csv`, `shutil`, `json` - 标准库

### 外部工具
- **pcg_bbox_single** - C++ 可执行文件，计算 UOBB
  - 位置: `LM2PCG/build/pcg_bbox_single`
  - 输入: PLY 文件路径
  - 输出: JSON 格式的几何信息

### API 配置
- **配置文件**: `LM2PCG/data/configs/.env`
- **必需环境变量**:
  ```
  API_KEY=<Azure OpenAI API Key>
  ENDPOINT=https://azure-openai-scanplan.openai.azure.com/
  ```

---

## 错误处理

### 脚本级错误
| 错误类型 | 检查点 | 处理方式 |
|---------|-------|---------|
| 未找到 cropped 图片 | `find_latest_cropped_image()` | 返回 False，输出错误信息 |
| API 调用失败 | `get_semantic_labels()` | 捕获异常，返回错误结果 |
| 未找到房间 | `find_room_for_panorama()` | 返回 None，终止集成 |
| UOBB 计算失败 | `compute_uobb()` | 返回 None，但保留 cluster 文件 |
| CSV 写入失败 | `append_to_csv()` | 抛出异常，回滚操作 |

### 网络级错误
| 错误类型 | 触发条件 | 前端显示 |
|---------|---------|---------|
| 脚本未找到 | 文件路径错误 | `Script not found: ...` |
| 超时 | 执行超过 60 秒 | `Script timeout (>60s)` |
| 返回码非 0 | 脚本执行失败 | `Integration failed` + stderr |

---

## 测试场景

### 正常流程
1. 在全景图上点击 5 个点选择一个门
2. 等待 SAM2 + mask2cluster 处理完成
3. 点击 "💾 Save New Object"
4. 等待 AI 标注（约 3-5 秒）
5. 看到成功弹窗，显示 "door" 标签
6. 检查 CSV 文件，确认新行已添加

### 边缘情况
- **重复点击**: 按钮在处理时禁用，防止重复提交
- **无 cropped 图片**: 脚本返回错误，前端显示提示
- **API 失败**: 超时或错误，前端显示详细信息
- **房间未找到**: 全景图文件名不匹配任何房间

---

## 未来改进

- [ ] 添加撤销功能（删除最后添加的对象）
- [ ] 支持手动编辑语义标签（而不是完全依赖 AI）
- [ ] 批量处理多个对象
- [ ] 实时预览集成结果（不需要刷新页面）
- [ ] 集成日志查看（显示详细的处理步骤）

---

**实现日期**: 2025年11月6日  
**作者**: Jackson Ye  
**测试状态**: ✅ 代码完成，待测试
