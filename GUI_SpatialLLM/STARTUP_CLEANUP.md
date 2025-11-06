# GUI Startup Cleanup

## 功能说明

GUI 启动时会自动执行以下清理操作：

### 1. 清理 SAM23D/outputs
- 删除整个 `SAM23D/outputs` 目录
- 重新创建空目录
- 目的：清除之前的分割结果（overlay、mask、cropped 图片、test.ply 等）

### 2. 清理所有 userselected 文件夹
- 搜索 `data/output/floor_*/room_*/results/filtered_clusters/`
- 删除所有 `userselected_*` 文件夹
- 目的：清除用户之前手动选择的点云片段

## 实现位置

**文件**: `GUI_SpatialLLM/GUI_streamlit.py` (第 20-55 行)

```python
# Clean SAM23D/outputs directory and userselected folders on startup
if 'outputs_cleaned' not in st.session_state:
    import shutil
    from pathlib import Path
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(current_dir)
    
    # Clean SAM23D/outputs
    outputs_dir = Path(root_dir) / "SAM23D" / "outputs"
    if outputs_dir.exists():
        shutil.rmtree(outputs_dir)
        outputs_dir.mkdir(parents=True, exist_ok=True)
    
    # Clean all userselected_* folders in data/output
    data_output_dir = Path(root_dir) / "data" / "output"
    if data_output_dir.exists():
        userselected_count = 0
        for floor_dir in data_output_dir.glob("floor_*"):
            if not floor_dir.is_dir():
                continue
            for room_dir in floor_dir.glob("room_*"):
                if not room_dir.is_dir():
                    continue
                results_dir = room_dir / "results" / "filtered_clusters"
                if results_dir.exists():
                    for userselected_dir in results_dir.glob("userselected_*"):
                        if userselected_dir.is_dir():
                            shutil.rmtree(userselected_dir)
                            userselected_count += 1
        
        if userselected_count > 0:
            print(f"✅ Cleaned {userselected_count} userselected folder(s)")
    
    st.session_state.outputs_cleaned = True
```

## 行为说明

### 触发时机
- **仅在 GUI 首次加载时执行**
- 使用 Streamlit session state 确保不重复执行
- 刷新页面或重新连接不会触发清理

### 清理范围
```
SAM23D/
└── outputs/                    # ❌ 整个目录删除并重建
    ├── *_overlay.png
    ├── *_binary_mask.png
    ├── *_cropped.png
    └── filtered_point_clouds/
        └── */
            └── *_test.ply

data/output/
├── floor_0/
│   ├── room_001/
│   │   └── results/
│   │       └── filtered_clusters/
│   │           └── userselected_001/    # ❌ 删除
│   ├── room_002/
│   │   └── results/
│   │       └── filtered_clusters/
│   │           └── userselected_002/    # ❌ 删除
│   └── room_003/
│       └── results/
│           └── filtered_clusters/
│               └── userselected_003/    # ❌ 删除
└── floor_1/
    └── ...
```

### 保留的内容
- ✅ `data/output/floor_*/room_*/` 中的其他文件（panorama、PLY、CSV 等）
- ✅ `results/filtered_clusters/` 中的其他子文件夹（非 userselected_* 的）
- ✅ 所有 room CSV 文件（但会包含已删除对象的记录）

## 注意事项

### ⚠️ CSV 记录不会被删除
虽然 `userselected_*` 文件夹被删除，但 CSV 文件中的对应行**不会被自动删除**。

例如：`room_002.csv` 中的这一行仍然存在：
```csv
0-2-14,14,2,0,door,0-2-14_door_cluster.ply,0,9.1616,10.5249,0.917777,1.15472,0.614599,1.5238,-0.0800158
```

但文件 `0-2-14_door_cluster.ply` 已被删除。

### 影响
- CSV 中会有"孤儿"记录（指向不存在的文件）
- 如果需要完全重置，应该手动清理 CSV 或重新生成数据集

## 测试

### 手动测试清理功能
```bash
cd GUI_SpatialLLM
python3 test_cleanup.py
```

### 验证清理结果
```bash
# 检查是否有残留的 userselected 文件夹
find /Users/jacksonye/SpatiaLLM-final/data/output -type d -name "userselected_*"

# 应该没有输出（表示已清理干净）
```

## 为什么需要这个功能？

### 使用场景
1. **新会话开始**: 用户开始新的标注任务，不希望看到旧数据
2. **演示/测试**: 快速重置环境，展示干净的状态
3. **避免混淆**: 防止旧的用户选择干扰新的工作流程

### 工作流程
```
启动 GUI
    ↓
自动清理 SAM23D/outputs + userselected folders
    ↓
用户在全景图上点击 5 个点
    ↓
SAM2 生成 mask 和 cropped 图片
    ↓
semantic_labeler.py 标注并集成
    ↓
创建新的 userselected_XXX 文件夹
    ↓
用户继续标注更多对象...
```

## 未来改进

- [ ] 添加清理 CSV 中孤儿记录的选项
- [ ] GUI 中添加"手动清理"按钮
- [ ] 清理前弹出确认对话框（可选）
- [ ] 记录清理日志到文件

---

**实现日期**: 2025年11月6日
**测试状态**: ✅ 已验证
