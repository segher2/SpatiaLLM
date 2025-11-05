#!/usr/bin/env python3
"""
AI Agent Helper - 完整的交互式工作流

流程：
1. AI Agent 运行 VIS/RMS 命令打开 viewer
2. 等待用户在 viewer 中选择物体并 Confirm
3. 自动关闭 viewer
4. 返回用户选择的物体代码
5. AI Agent 可以对这些物体执行后续操作（CLR、RCN、VOL 等）
"""

import json
import subprocess
import time
from pathlib import Path
from typing import List, Dict, Optional
import signal
import os

class AIAgentHelper:
    """AI Agent 助手类，用于管理 viewer 交互和选择获取"""
    
    def __init__(self, root: Optional[Path] = None):
        from ai_api import Dispatcher
        self.dispatcher = Dispatcher(root)
        self.root = self.dispatcher.root
        self.selection_file = Path("/tmp/viewer_selection.json")
        self.viewer_processes = []
        
    def visualize_and_wait_for_selection(
        self,
        mode: str = 'room',
        room_codes: Optional[List[str]] = None,
        object_codes: Optional[List[str]] = None,
        timeout: int = 300,
        auto_close: bool = True
    ) -> List[str]:
        """
        可视化并等待用户选择
        
        Args:
            mode: 可视化模式 ('room', 'clusters', 'multi-rooms')
            room_codes: 房间代码列表 (例如 ['0-7'])
            object_codes: 物体代码列表 (例如 ['0-7-12'])
            timeout: 等待超时时间（秒），默认 5 分钟
            auto_close: 是否在用户选择后自动关闭 viewer
            
        Returns:
            用户选择的物体代码列表（例如 ['0-7', '0-3']）
        """
        print("\n" + "="*60)
        print("🚀 AI Agent 工作流程")
        print("="*60)
        
        # 清除旧的选择文件
        if self.selection_file.exists():
            self.selection_file.unlink()
            
        # 1. 启动 viewer
        print(f"\n📺 步骤 1: 启动 viewer (mode={mode})...")
        name = f"ai_agent_{int(time.time())}"
        
        result = self.dispatcher.op_VIS(
            mode=mode,
            name=name,
            room_codes=room_codes,
            object_codes=object_codes,
            auto_serve=True,
            port=5173
        )
        
        print(f"   ✓ Viewer 已启动")
        print(f"   📍 URL: {result['viewer_url']}")
        print(f"   🔌 API: http://localhost:8090")
        
        # 记录进程信息以便后续关闭
        self._store_server_pids()
        
        # 2. 等待用户选择
        print(f"\n⏳ 步骤 2: 等待用户选择...")
        print(f"   请在浏览器中:")
        print(f"   1. 点击要选择的物体")
        print(f"   2. 点击 'Confirm All' 按钮")
        print(f"   3. 等待自动关闭 (或超时 {timeout}秒)")
        print()
        
        selected_codes = self._wait_for_selection(timeout)
        
        # 3. 自动关闭 viewer
        if auto_close:
            print(f"\n🛑 步骤 3: 自动关闭 viewer...")
            self._close_viewer()
            print(f"   ✓ Viewer 已关闭")
        
        # 4. 返回结果
        print(f"\n✅ 完成!")
        if selected_codes:
            print(f"   用户选择了 {len(selected_codes)} 个物体:")
            for code in selected_codes:
                print(f"      - {code}")
        else:
            print(f"   用户未选择任何物体")
        
        print("="*60 + "\n")
        
        return selected_codes
    
    def _wait_for_selection(self, timeout: int) -> List[str]:
        """等待用户在 viewer 中完成选择"""
        start_time = time.time()
        last_check_time = start_time
        
        while time.time() - start_time < timeout:
            # 每秒检查一次
            time.sleep(1)
            
            # 每 10 秒打印一次等待状态
            current_time = time.time()
            if current_time - last_check_time >= 10:
                elapsed = int(current_time - start_time)
                remaining = timeout - elapsed
                print(f"   ⏱️  已等待 {elapsed}秒，剩余 {remaining}秒...")
                last_check_time = current_time
            
            # 检查选择文件是否存在
            if self.selection_file.exists():
                try:
                    with open(self.selection_file, 'r') as f:
                        data = json.load(f)
                    
                    # 打印友好信息
                    print("\n" + "="*60)
                    print("🎯 实时检测到用户选择！")
                    print("="*60)
                    
                    if len(data) == 0:
                        print("📭 用户清空了选择")
                        return []
                    
                    print(f"✅ 用户选择了 {len(data)} 个物体：\n")
                    codes = []
                    for i, item in enumerate(data, 1):
                        display_name = item.get('displayName', 'unknown')
                        item_code = item.get('itemCode', 'unknown')
                        print(f"   {i}. {display_name} ({item_code})")
                        codes.append(item_code)
                    
                    print("\n📦 详细信息：")
                    print(json.dumps(data, indent=2, ensure_ascii=False))
                    print("="*60)
                    
                    return codes
                    
                except Exception as e:
                    print(f"   ⚠️  读取选择文件失败: {e}")
                    continue
        
        print(f"\n⏱️  超时! 用户在 {timeout} 秒内未完成选择")
        return []
    
    def _store_server_pids(self):
        """记录服务器进程 ID"""
        pid_file = Path("/tmp/dev_servers.pid")
        if pid_file.exists():
            try:
                pids = pid_file.read_text().strip().split()
                self.viewer_processes = [int(pid) for pid in pids]
            except:
                pass
    
    def _close_viewer(self):
        """关闭 viewer 服务器"""
        stop_script = self.root / "web" / "pointcloud-viewer" / "stop_dev.sh"
        if stop_script.exists():
            subprocess.run(['bash', str(stop_script)], 
                          stdout=subprocess.DEVNULL, 
                          stderr=subprocess.DEVNULL)
        else:
            # 备选方案：直接杀进程
            if self.viewer_processes:
                for pid in self.viewer_processes:
                    try:
                        os.kill(pid, signal.SIGTERM)
                    except:
                        pass

    def execute_operations_on_selection(
        self,
        selected_codes: List[str],
        operations: List[str]
    ) -> Dict[str, any]:
        """
        对用户选择的物体执行操作
        
        Args:
            selected_codes: 用户选择的物体代码列表
            operations: 要执行的操作列表，例如 ['CLR', 'VOL', 'RCN']
            
        Returns:
            操作结果字典
        """
        if not selected_codes:
            print("⚠️  没有选择任何物体，无法执行操作")
            return {}
        
        print(f"\n🔧 对 {len(selected_codes)} 个物体执行操作...")
        results = {}
        
        for op in operations:
            print(f"\n📌 执行 {op} 操作...")
            op_results = []
            
            for code in selected_codes:
                try:
                    print(f"   处理 {code}...")
                    
                    if op == 'CLR':
                        result = self.dispatcher.op_CLR(object_code=code)
                        op_results.append({'code': code, 'result': result})
                        print(f"      ✓ 颜色分析完成")
                        
                    elif op == 'VOL':
                        mesh_path, volume, is_closed = self.dispatcher.op_VOL(
                            object_code=code, 
                            auto_reconstruct=True
                        )
                        op_results.append({
                            'code': code,
                            'volume': volume,
                            'closed': is_closed,
                            'mesh': str(mesh_path)
                        })
                        print(f"      ✓ 体积: {volume:.2f}, 封闭: {is_closed}")
                        
                    elif op == 'RCN':
                        mesh_path = self.dispatcher.op_RCN(object_code=code)
                        op_results.append({'code': code, 'mesh': str(mesh_path)})
                        print(f"      ✓ 重建完成: {mesh_path.name}")
                        
                    elif op == 'BBD':
                        # BBD 需要两个物体，这里跳过
                        print(f"      ⚠️  BBD 需要两个物体，跳过")
                        
                    else:
                        print(f"      ⚠️  未知操作: {op}")
                        
                except Exception as e:
                    print(f"      ❌ 失败: {e}")
                    op_results.append({'code': code, 'error': str(e)})
            
            results[op] = op_results
        
        print(f"\n✅ 所有操作完成!")
        return results


# 便捷函数
def interactive_workflow(
    mode: str = 'room',
    room_codes: Optional[List[str]] = None,
    operations: Optional[List[str]] = None,
    timeout: int = 300
) -> Dict[str, any]:
    """
    一键式交互工作流
    
    示例:
        # 可视化房间 0-7，等待用户选择，然后执行颜色分析
        results = interactive_workflow(
            mode='room',
            room_codes=['0-7'],
            operations=['CLR', 'VOL']
        )
    """
    helper = AIAgentHelper()
    
    # 1. 可视化并等待选择
    selected_codes = helper.visualize_and_wait_for_selection(
        mode=mode,
        room_codes=room_codes,
        timeout=timeout,
        auto_close=True
    )
    
    # 2. 执行操作
    results = {}
    if selected_codes and operations:
        results = helper.execute_operations_on_selection(
            selected_codes=selected_codes,
            operations=operations
        )
    
    return {
        'selected_codes': selected_codes,
        'operations': results
    }


if __name__ == '__main__':
    # 示例用法
    print("AI Agent Helper - 交互式工作流示例\n")
    
    # 示例 1: 可视化房间并等待选择
    print("示例 1: 可视化房间 0-7，等待用户选择")
    print("-" * 60)
    
    helper = AIAgentHelper()
    selected = helper.visualize_and_wait_for_selection(
        mode='room',
        room_codes=['0-7'],
        timeout=300
    )
    
    if selected:
        # 示例 2: 对选择的物体执行操作
        print("\n示例 2: 对选择的物体执行颜色分析")
        print("-" * 60)
        results = helper.execute_operations_on_selection(
            selected_codes=selected,
            operations=['CLR']
        )
        
        print("\n最终结果:")
        print(json.dumps(results, indent=2, ensure_ascii=False))
