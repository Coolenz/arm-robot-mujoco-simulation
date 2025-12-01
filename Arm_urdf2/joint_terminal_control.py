#!/usr/bin/env python3
"""
关节控制串口程序 - 支持串口控制和MuJoCo control窗口motor控制
支持两种控制模式：终端输入和MuJoCo control窗口motor控制
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
target_positions[1] = -1.4  # J2关节原点位置设为-1.4弧度
control_enabled = True
last_command_time = time.time()
control_mode = "serial"  # 控制模式: "serial"=串口控制, "manual"=control窗口控制
last_manual_angles = [0.0] * model.njnt  # 上次手动控制时的角度
manual_target_reached = False  # 手动控制目标到达标志
last_print_time = time.time()  # 上次打印时间

def monitor_and_control():
    """监视和控制主循环"""
    global control_enabled, target_positions, last_command_time, control_mode, last_manual_angles, manual_target_reached, last_print_time
    
    # 状态跟踪变量
    target_reached = False
    last_angles = [0.0] * model.njnt
    
    while control_enabled:
        # 获取当前关节角度（弧度）
        current_angles = []
        for i in range(model.njnt):
            angle_rad = data.qpos[i]
            current_angles.append(angle_rad)
        
        # 检查是否到达目标位置（串口控制模式）
        all_reached = True
        for i in range(model.njnt):
            if abs(current_angles[i] - target_positions[i]) > 0.05:  # 0.05弧度容差（约2.86°）
                all_reached = False
                break
        
        # 如果到达目标位置且之前未打印（串口控制模式）
        if all_reached and not target_reached and control_mode == "serial":
            # 打印最终关节角度
            status_line = f"关节当前位置："
            for i in range(model.njnt):
                status_line += f"J{i+1}:{current_angles[i]:6.3f} "
            print(status_line, flush=True)
            target_reached = True
            print("\n> ", end='', flush=True)
        
        # 如果角度发生变化，重置目标到达状态
        if not all_reached:
            target_reached = False
        
        # 检查control窗口控制是否到达稳定状态（手动控制模式）
        if control_mode == "manual":
            # 每秒打印一次关节电机位置（使用独立的打印时间变量）
            current_time = time.time()
            if current_time - last_print_time >= 1.0:
                # 打印关节电机位置
                status_line = f"关节电机位置："
                for i in range(model.njnt):
                    status_line += f"J{i+1}:{current_angles[i]:6.3f} "
                print(status_line, flush=True)
                last_print_time = current_time
            
            control_stable = True
            for i in range(model.njnt):
                if abs(current_angles[i] - last_manual_angles[i]) > 0.01:  # 检查角度是否稳定
                    control_stable = False
                    break
            
            # 如果control窗口控制到达稳定状态且之前未打印
            if control_stable and not manual_target_reached:
                # 打印最终关节角度
                status_line = f"关节当前位置："
                for i in range(model.njnt):
                    status_line += f"J{i+1}:{current_angles[i]:6.3f} "
                print(status_line, flush=True)
                manual_target_reached = True
                print("\n> ", end='', flush=True)
            
            # 更新上次control窗口控制角度
            last_manual_angles = current_angles.copy()
        
        # 简单的输入检查（非阻塞）
        try:
            # 检查是否有输入（使用select实现非阻塞输入）
            import select
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                user_input = sys.stdin.readline().strip()
                last_command_time = time.time()
                
                if user_input.lower() == 'q':
                    print("正在退出控制...")
                    control_enabled = False
                    break
                
                # 处理控制模式切换命令
                if user_input == "m1":
                    # 切换到串口控制模式
                    control_mode = "serial"
                    print("📡 切换到串口控制模式")
                    print("> ", end='', flush=True)
                    continue
                elif user_input == "m2":
                    # 切换到control窗口控制模式
                    control_mode = "manual"
                    manual_target_reached = False  # 重置手动控制目标到达标志
                    print("🔧 切换到Control窗口控制模式")
                    print("> ", end='', flush=True)
                    continue
                
                # 解析控制输入 - 支持5个数值格式（仅在串口控制模式下）
                if control_mode == "serial":
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
                else:
                    print(f"当前为Control窗口控制模式，请输入'm1'切换到串口控制模式")
                    
        except KeyboardInterrupt:
            print("\n控制中断")
            control_enabled = False
            break
        except Exception:
            pass
        
        time.sleep(0.1)  # 100ms更新频率

def apply_control():
    """应用控制信号到关节"""
    global control_mode
    
    # 如果在control窗口控制模式下，完全禁用自动控制
    # 让control窗口的控制信号直接生效
    if control_mode == "manual":
        return
    
    # 简单的位置控制（串口控制模式）
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
    
    print("🚀 双模式控制程序启动")
    print("支持两种控制模式：")
    print("1. 串口控制模式 - 通过终端输入控制关节位置")
    print("2. Control窗口控制模式 - 通过MuJoCo control窗口控制motor")
    print("\n使用以下命令手动切换控制模式：")
    print("  • 输入 'm1' 切换到串口控制模式")
    print("  • 输入 'm2' 切换到Control窗口控制模式")
    print("\n关节到达目标位置后将自动打印当前位置")
    print("\n> ", end='', flush=True)

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