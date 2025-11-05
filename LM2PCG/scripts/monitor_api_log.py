#!/usr/bin/env python3
"""
实时监控 API 服务器日志，显示用户选择
"""
import time
import subprocess

print("🔍 正在监控 API 服务器日志...")
print("📝 日志文件: /tmp/api_server.log")
print("按 Ctrl+C 停止监控\n")
print("="*60)

# 使用 tail -f 持续监控日志
try:
    proc = subprocess.Popen(
        ['tail', '-f', '/tmp/api_server.log'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    
    for line in proc.stdout:
        print(line, end='')
        
except KeyboardInterrupt:
    print("\n\n监控已停止")
    proc.terminate()
