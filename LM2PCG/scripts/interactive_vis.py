#!/usr/bin/env python3
"""
简化的 AI Agent 交互式命令
用法：
    python3 scripts/interactive_vis.py VIS 0-7
    python3 scripts/interactive_vis.py RMS
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from ai_agent_helper import AIAgentHelper
from ai_api import Dispatcher, auto_detect_vis_mode, generate_vis_name

def main():
    if len(sys.argv) < 2:
        print("用法:")
        print("  python3 scripts/interactive_vis.py VIS <room-codes>")
        print("  python3 scripts/interactive_vis.py RMS")
        print()
        print("示例:")
        print("  python3 scripts/interactive_vis.py VIS 0-7")
        print("  python3 scripts/interactive_vis.py VIS 0-7 0-3")
        print("  python3 scripts/interactive_vis.py RMS")
        sys.exit(1)
    
    command = sys.argv[1].upper()
    helper = AIAgentHelper()
    
    if command == 'VIS':
        if len(sys.argv) < 3:
            print("错误: VIS 命令需要至少一个房间代码")
            print("示例: python3 scripts/interactive_vis.py VIS 0-7")
            sys.exit(1)
        
        # 解析房间代码
        codes = sys.argv[2:]
        
        # 自动检测模式
        mode, room_codes, object_codes = auto_detect_vis_mode(codes)
        
        print(f"\n检测到模式: {mode}")
        print(f"房间代码: {room_codes}")
        if object_codes:
            print(f"物体代码: {object_codes}")
        
        # 可视化并等待选择
        selected = helper.visualize_and_wait_for_selection(
            mode=mode,
            room_codes=room_codes,
            object_codes=object_codes,
            timeout=300,
            auto_close=True
        )
        
        # 返回选择结果
        if selected:
            print("\n" + "="*60)
            print("✅ 用户选择完成！")
            print("="*60)
            print(f"选择的物体代码: {selected}")
            print()
            print("你可以使用这些代码执行后续操作：")
            print(f"  CLR {' '.join(selected)}")
            print(f"  VOL {' '.join(selected)}")
            print(f"  RCN {' '.join(selected)}")
            print()
            
            # 返回代码供脚本使用
            return selected
        else:
            print("\n⚠️  用户未选择任何物体")
            return []
    
    elif command == 'RMS':
        # 显示所有房间并可视化
        dispatcher = Dispatcher()
        result = dispatcher.op_RMS(visualize=True)
        
        print("\n" + "="*60)
        print("📊 房间清单已生成")
        print("="*60)
        
        # 询问用户是否要选择房间进行详细查看
        print("\n提示: 你可以选择特定房间进行详细可视化")
        print("示例: python3 scripts/interactive_vis.py VIS 0-7")
        
        return []
    
    else:
        print(f"错误: 未知命令 '{command}'")
        print("支持的命令: VIS, RMS")
        sys.exit(1)

if __name__ == '__main__':
    selected_codes = main()
    
    # 如果作为模块导入，可以返回选择的代码
    if selected_codes:
        # 保存到环境变量或文件供后续使用
        import json
        output_file = Path("/tmp/last_selection.json")
        with open(output_file, 'w') as f:
            json.dump({
                'codes': selected_codes,
                'timestamp': __import__('datetime').datetime.now().isoformat()
            }, f, indent=2)
        print(f"💾 选择已保存到: {output_file}")
