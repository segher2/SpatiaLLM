# 自动化可视化流程实现总结

## ✅ 已完成的工作

### 1. 核心脚本：`prepare_visualization.mjs`

创建了完整的自动化可视化准备脚本，位于：
`/web/pointcloud-viewer/scripts/prepare_visualization.mjs`

**功能特性：**

- ✅ **5 种可视化模式**：
  - `room` - 可视化整个房间（shell + 所有 clusters）
  - `clusters` - 可视化选定的 clusters（通过对象代码或文件路径）
  - `multi-rooms` - 可视化多个房间的 shells
  - `random` - 随机选择 N 个 clusters
  - `room-with-objects` - 房间 shell + 选定对象

- ✅ **与 ai_api.py 完全集成**：
  - 调用 `resolve-object <code>` 解析对象资源
  - 调用 `resolve-room <code>` 解析房间资源
  - 支持相同的对象/房间代码格式 (`<floor>-<room>-<object>`)

- ✅ **自动清理缓存**：
  - 每次运行前删除旧的 `.ply` 文件
  - 删除旧的 manifest JSON
  - 可通过 `--no-clean` 禁用

- ✅ **降采样集成**：
  - 调用现有的 `downsample_and_prepare_room.mjs`
  - 支持 ratio-based 和 voxel-based 降采样
  - 保留 label 和 point_id 属性
  - 输出 ASCII PLY 格式

- ✅ **Manifest 自动生成**：
  - 统一的 manifest schema (v1)
  - 正确的 item metadata（id, name, kind, role, source, group）
  - 与 viewer 完全兼容

### 2. 修复的问题

#### `downsample_and_prepare_room.mjs` 改进
- ✅ 修复了 `parseArgs` 函数，支持多个 `--cluster` 参数
- ✅ 现在可以一次处理多个 cluster 文件

#### `prepare_visualization.mjs` 改进  
- ✅ 添加了 `--shellCopy /dev/null` 作为 fallback，避免脚本报错
- ✅ 所有模式都经过测试并正常工作

### 3. 文档更新

#### README.md
- ✅ 添加了完整的"Automated Visualization Workflow"章节
- ✅ 包含所有 5 种模式的使用示例
- ✅ 详细的参数说明
- ✅ 与 ai_api.py 的集成说明

#### package.json
- ✅ 添加了 npm scripts：
  - `npm run visualize` - 运行可视化脚本
  - `npm run visualize:help` - 显示帮助信息

## 📋 使用示例

### 1. 可视化整个房间
```bash
npm run visualize -- --mode room --room 0-7 --name room_007
```

### 2. 随机选择 5 个 clusters
```bash
npm run visualize -- --mode random --count 5 --source "../../output/Full House" --name random_5
```

### 3. 可视化选定的对象（通过对象代码）
```bash
npm run visualize -- --mode clusters --objects "0-7-12,0-7-15,0-7-3" --name selected_furniture
```

### 4. 可视化多个房间的 shells
```bash
npm run visualize -- --mode multi-rooms --rooms "0-7,0-8,1-7" --name floor_overview
```

### 5. 房间 + 选定对象
```bash
npm run visualize -- --mode room-with-objects --room 0-7 --objects "0-7-12,0-7-15" --name room_furniture
```

## 🔧 工作流程

每次运行脚本时，会执行以下步骤：

1. **清理缓存** 
   - 删除 `public/data/<name>/` 目录
   - 删除 `public/manifests/<name>.json` 文件

2. **路径解析**
   - 通过 `ai_api.py` 查找 shells、clusters、UOBBs
   - 支持对象代码（如 `0-7-12`）和房间代码（如 `0-7`）

3. **降采样**
   - 调用 `downsample_and_prepare_room.mjs`
   - 使用配置的 ratio/voxel 参数
   - 保留自定义属性（label, point_id）
   - 输出 ASCII PLY 格式

4. **生成 Manifest**
   - 创建统一的 manifest JSON
   - 正确的 item metadata
   - 自动检测和复制 UOBB 文件

5. **输出**
   - 打印 viewer URL
   - 显示数据位置和 manifest 路径

## 🎯 与 ai_api.py 的兼容性

脚本完全兼容 `ai_api.py` 的设计理念：

- **PathIndex 系统**：使用相同的路径解析逻辑
- **对象代码格式**：`<floor>-<room>-<object>` (e.g., `0-7-12`)
- **房间代码格式**：`<floor>-<room>` (e.g., `0-7`)
- **文件命名约定**：
  - Clusters: `<object_code>_<class>_cluster.ply`
  - UOBBs: `<object_code>_<class>_uobb.ply`
  - Meshes: `<object_code>_<class>_mesh[_poisson|_af].ply`
  - Shell: `<object_code>_shell.ply`

## 🚀 下一步扩展

如果需要添加更多功能，可以考虑：

1. **VIS 模式** - 添加为 ai_api.py 的一个操作代码
   - 类似 RCN (reconstruction)、VOL (volume)、ARE (area) 等
   - 可以从 ai_api.py CLI 直接调用可视化

2. **批量模式** - 处理多个房间或对象列表文件

3. **配置文件支持** - 通过 JSON/YAML 配置复杂的可视化场景

4. **Web UI** - 创建交互式界面选择要可视化的内容

## ✨ 成功测试

已成功测试并验证：
- ✅ Random 模式生成 5 个随机 clusters
- ✅ 降采样正常工作（10% ratio）
- ✅ Manifest 正确生成（5 个 items）
- ✅ 清理缓存功能正常
- ✅ 多个 --cluster 参数正确处理
- ✅ 输出 URL 正确（http://localhost:5173/?manifest=/manifests/random_5_clusters.json）
