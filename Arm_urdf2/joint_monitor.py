#!/usr/bin/env python3
"""
关节位置监视器 - 实时显示所有关节角度
基于open_viewer.py改进，添加关节监视功能
"""
import mujoco
import mujoco.viewer
import os
import time
import threading

# 加载模型
model = mujoco.MjModel.from_xml_path("arm1.xml")
data = mujoco.MjData(model)

# 关节信息
print("=== 关节监视器启动 ===")
print(f"模型: arm1.xml")
print(f"关节数量: {model.njnt}")
print(f"关节名称: {[model.joint(i).name for i in range(model.njnt)]}")
print("=" * 40)

def monitor_joints():
    """关节监视线程函数"""
    while True:
        # 获取所有关节位置
        joint_positions = []
        for i in range(model.njnt):
            # 转换为角度（弧度转度）
            angle_rad = data.qpos[i]
            angle_deg = angle_rad * 180.0 / 3.14159
            joint_positions.append(f"{model.joint(i).name}: {angle_deg:6.1f}°")
        
        # 简洁的并列显示格式
        print(f"\r{' | '.join(joint_positions)}", end='', flush=True)
        
        time.sleep(1.0)  # 1秒更新一次

def main():
    # 启动关节监视线程
    monitor_thread = threading.Thread(target=monitor_joints, daemon=True)
    monitor_thread.start()
    
    # 启动查看器
    print("\n正在启动MuJoCo查看器...")
    viewer = mujoco.viewer.launch_passive(model, data)

    print("\n模型信息:")
    print(f"关节: {model.njnt}个")
    print(f"刚体: {model.nbody}个") 
    print(f"几何体: {model.ngeom}个")
    print(f"自由度: {model.nq}")
    print("\n操作提示:")
    print("- 鼠标左键拖拽: 旋转视角")
    print("- 鼠标滚轮: 缩放")
    print("- 鼠标右键拖拽: 平移")
    print("- ESC键: 退出查看器")
    print("\n关节位置监视器已启动，每秒更新一次...")
    print("=" * 60)
    
    # 主循环
    try:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)  # 稍微降低CPU占用
    except KeyboardInterrupt:
        print("\n\n用户中断，正在关闭...")
    finally:
        print("查看器已关闭")

if __name__ == "__main__":
    main()