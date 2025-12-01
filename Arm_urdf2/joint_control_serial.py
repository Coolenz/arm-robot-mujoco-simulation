#!/usr/bin/env python3
"""
简单终端关节控制器 - 通过终端输入控制MuJoCo关节角度
使用弧度制显示，支持直接输入5个数值
"""
import mujoco
import mujoco.viewer
import os
import time
import threading
import sys

# 加载模型
model = mujoco.MjModel.from_xml_path("arm1.xml")
data = mujoco.MjData(model)

# 全局变量
target_positions = [0.0] * model.njnt  # 目标位置（弧度）
control_enabled = True
last_command_time = time.time()

def monitor_and_control():
    """监视和控制主循环"""
    global control_enabled, target_positions, last_command_time
    
    # 状态跟踪变量
    target_reached = False
    last_angles = [0.0] * model.njnt
    
    while control_enabled:
        # 获取当前关节角度（弧度）
        current_angles = []
        for i in range(model.njnt):
            angle_rad = data.qpos[i]
            current_angles.append(angle_rad)
        
        # 检查是否到达目标位置
        all_reached = True
        for i in range(model.njnt):
            if abs(current_angles[i] - target_positions[i]) > 0.05:  # 0.05弧度容差（约2.86°）
                all_reached = False
                break
        
        # 如果到达目标位置且之前未打印
        if all_reached and not target_reached:
            # 打印最终关节角度
            status_line = f"关节当前位置："
            for i in range(model.njnt):
                status_line += f"J{i+1}:{current_angles[i]:6.3f} "
            print(status_line, flush=True)
            target_reached = True
            
            # 提示用户输入新命令
            print("\n> ", end='', flush=True)
        
        # 如果角度发生变化，重置目标到达状态
        if not all_reached:
            target_reached = False
        
        # 简单的输入检查（非阻塞）
        try:
            # 检查是否有输入
            if target_reached or time.time() - last_command_time > 2.0:
                user_input = input().strip()
                last_command_time = time.time()
                
                if user_input.lower() == 'q':
                    print("正在退出控制...")
                    control_enabled = False
                    break
                
                # 解析控制输入 - 支持5个数值格式
                try:
                    parts = user_input.split(',')
                    
                    # 检查是否是5个数值的格式
                    if len(parts) == 5:
                        # 解析5个数值并自动转换为弧度
                        for i in range(5):
                            angle_value = float(parts[i].strip())
                            
                            # 检查度数范围（所有关节使用-180°~180°范围）
                            if angle_value < -180.0 or angle_value > 180.0:
                                print(f"❌ 关节{i+1}超出范围！有效范围: -180° ~ 180°")
                                continue
                            
                            # 自动将度数转换为弧度
                            target_positions[i] = angle_value * 3.14159 / 180.0
                        
                        # 不打印设置确认信息
                        target_reached = False  # 重置到达状态
                    else:
                        # 尝试解析单个关节格式
                        parts = user_input.split()
                        if len(parts) == 2:
                            # 关节编号从1开始，需要转换为0开始的内部索引
                            joint_num = int(parts[0])
                            joint_idx = joint_num - 1
                            target_angle = float(parts[1])
                            
                            # 验证关节编号
                            if joint_num < 1 or joint_num > model.njnt:
                                print(f"❌ 关节编号错误！有效范围: 1-{model.njnt}")
                                continue
                            
                            # 检查度数范围（所有关节使用-180°~180°范围）
                            if target_angle < -180.0 or target_angle > 180.0:
                                print(f"❌ 关节{joint_num}超出范围！有效范围: -180° ~ 180°")
                                continue
                            
                            # 自动将度数转换为弧度
                            target_positions[joint_idx] = target_angle * 3.14159 / 180.0
                            
                            # 不打印设置确认信息
                            target_reached = False  # 重置到达状态
                        else:
                            print("❌ 格式错误！请输入: 关节编号 目标角度 或 5个弧度值用逗号分隔")
                    
                except ValueError:
                    print("❌ 输入格式错误！请输入数字")
                except Exception as e:
                    print(f"❌ 控制错误: {e}")
                    
        except KeyboardInterrupt:
            print("\n控制中断")
            control_enabled = False
            break
        except Exception:
            pass
        
        time.sleep(0.1)  # 100ms更新频率

def apply_control():
    """应用控制信号到关节"""
    # 简单的位置控制
    for i in range(min(model.njnt, model.nu)):  # 确保不超出执行器数量
        if i < model.nu:
            # 计算控制信号（简单的P控制）
            error = target_positions[i] - data.qpos[i]
            data.ctrl[i] = 20.0 * error  # 增益系数

def main():
    """主函数"""
    global control_enabled
    
    # 启动监视和控制线程
    control_thread = threading.Thread(target=monitor_and_control, daemon=True)
    control_thread.start()
    
    # 尝试启动查看器（macOS兼容版本）
    try:
        # 使用mjpython兼容的方式启动查看器
        viewer = mujoco.viewer.launch_passive(model, data)
        print("✅ MuJoCo查看器已启动！")
        
        # 主循环（带viewer版本）
        while viewer.is_running() and control_enabled:
            # 应用控制
            apply_control()
            
            # 运行物理仿真
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)  # 1kHz仿真频率
            
    except RuntimeError as e:
        if "mjpython" in str(e):
            print("⚠️  检测到macOS限制，使用无界面模式运行")
            print("💡 提示：请使用 'mjpython joint_control_simple.py' 来启动可视化界面")
            
            # 无界面模式主循环
            try:
                while control_enabled:
                    # 应用控制
                    apply_control()
                    
                    # 运行物理仿真
                    mujoco.mj_step(model, data)
                    time.sleep(0.001)  # 1kHz仿真频率
                    
            except KeyboardInterrupt:
                print("\n\n用户中断，正在关闭...")
        else:
            raise e
            
    except KeyboardInterrupt:
        print("\n\n用户中断，正在关闭...")
    finally:
        control_enabled = False
        print("程序已退出")

if __name__ == "__main__":
    main()