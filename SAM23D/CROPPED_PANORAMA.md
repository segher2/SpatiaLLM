# Cropped Panorama Generation

## 概述

在 SAM2 分割流程中，除了生成 overlay 和 binary mask，现在还会**自动生成裁剪后的全景图**。

## 功能说明

当用户在全景图上点击5个点触发 SAM2 分割后，系统会自动生成3个输出文件：

1. **`{filename}_overlay.png`** - 原图 + 半透明蓝色遮罩层
2. **`{filename}_binary_mask.png`** - 二值分割 mask
3. **`{filename}_cropped.png`** - 🆕 裁剪后的全景图

## 裁剪逻辑

```python
# 创建带透明通道的图像
cropped_pano = np.zeros((*original_image.shape[:2], 4), dtype=np.uint8)

# 只保留 mask 区域的像素
cropped_pano[best_mask > 0, :3] = original_image[best_mask > 0]  # RGB
cropped_pano[best_mask > 0, 3] = 255  # Alpha (不透明)

# mask 外的区域保持透明 (alpha = 0)
```

## 输出特点

- **格式**: PNG (支持透明通道)
- **尺寸**: 与原始全景图相同
- **内容**: 
  - Mask 内区域：保留原始像素
  - Mask 外区域：完全透明
- **位置**: `SAM23D/outputs/{filename}_cropped.png`

## 使用场景

1. **图像分析**: 单独查看分割出的目标物体
2. **数据标注**: 提取感兴趣区域
3. **后期处理**: 可以直接用于合成或其他处理
4. **可视化**: 在透明背景上展示分割结果

## 示例

假设全景图文件名为 `room_001.jpg`，输出文件为：

```
SAM23D/outputs/
├── room_001_overlay.png        # 原图 + 蓝色遮罩
├── room_001_binary_mask.png    # 黑白 mask
└── room_001_cropped.png        # 裁剪后的全景图 (带透明背景)
```

## 技术细节

- **无需额外配置**: 功能已集成到 `sam2_predictor.py`
- **自动生成**: 每次 SAM2 运行时自动创建
- **性能影响**: 几乎无影响（单次数组操作）
- **文件大小**: 通常比原图小（大量透明像素）

## 代码位置

- **实现文件**: `SAM23D/sam2_predictor.py` (第 122-128 行)
- **调用流程**: GUI → bridge_server → sam2_predictor → 自动生成
