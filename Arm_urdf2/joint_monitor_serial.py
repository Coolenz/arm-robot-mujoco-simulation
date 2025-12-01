#!/usr/bin/env python3
"""
关节角度串口发送监视器
通过串口将关节角度数据发送到ESP32
"""

import mujoco
import mujoco.viewer
import time
import threading
import serial
import serial.tools.list_ports
import math

class JointMonitorSerial:
    def __init__(self, port=None, baudrate=115200):
        """
        初始化串口关节监视器
        
        Args:
            port: 串口号，如 'COM3' 或 '/dev/ttyUSB0'
            baudrate: 波特率，默认115200
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.joint_angles = [0.0] * 5
        self.last_send_time = 0
        
    def find_serial_ports(self):
        """查找可用的串口"""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect_serial(self):
        """连接串口"""
        try:
            if self.port:
                # 使用指定串口
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"✅ 串口连接成功: {self.port} @ {self.baudrate}bps")
                return True
            else:
                # 自动查找串口
                available_ports = self.find_serial_ports()
                if available_ports:
                    self.port = available_ports[0]
                    self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
                    print(f"✅ 自动连接串口: {self.port} @ {self.baudrate}bps")
                    return True
                else:
                    print("❌ 未找到可用串口")
                    return False
        except Exception as e:
            print(f"❌ 串口连接失败: {e}")
            return False
    
    def format_joint_data(self, angles_deg):
        """
        格式化关节数据为串口发送格式
        
        Args:
            angles_deg: 角度列表（度数）
            
        Returns:
            格式化的字符串
        """
        # 格式: J1:90.0,J2:45.0,J3:0.0,J4:0.0,J5:0.0\n
        data_str = f"J1:{angles_deg[0]:.1f},J2:{angles_deg[1]:.1f},J3:{angles_deg[2]:.1f},J4:{angles_deg[3]:.1f},J5:{angles_deg[4]:.1f}\n"
        return data_str
    
    def send_joint_data(self, angles_deg):
        """发送关节数据到串口"""
        data_str = self.format_joint_data(angles_deg)
        
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(data_str.encode('utf-8'))
                print(f"📡 串口发送: {data_str.strip()}")
                return True
            except Exception as e:
                print(f"❌ 串口发送失败: {e}")
                return False
        else:
            # 模拟模式：显示发送数据但不实际发送
            print(f"📊 模拟发送: {data_str.strip()}")
            return True
    
    def monitor_joints(self, model, data):
        """监视关节角度的线程函数"""
        print("=== 关节串口监视器启动 ===")
        print(f"模型: {model}")
        print(f"关节数量: {model.njnt}")
        
        joint_names = []
        for i in range(model.njnt):
            joint_names.append(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i))
        print(f"关节名称: {joint_names}")
        
        if self.connect_serial():
            print("✅ 开始串口数据发送...")
            self.running = True
        else:
            print("⚠️  进入模拟模式：显示发送数据但不实际发送到串口")
            self.running = True
        
        while self.running:
            try:
                # 获取关节位置（弧度）
                joint_positions = []
                for i in range(model.njnt):
                    joint_positions.append(data.qpos[i])
                
                # 转换为角度
                angles_deg = [math.degrees(pos) for pos in joint_positions]
                self.joint_angles = angles_deg
                
                # 每100ms发送一次数据
                current_time = time.time()
                if current_time - self.last_send_time >= 0.1:  # 100ms间隔
                    self.send_joint_data(angles_deg)
                    self.last_send_time = current_time
                
                time.sleep(0.01)  # 10ms小延迟
                
            except Exception as e:
                print(f"❌ 监视线程错误: {e}")
                break
    
    def start_monitoring(self, model, data):
        """开始监视关节"""
        monitor_thread = threading.Thread(target=self.monitor_joints, args=(model, data))
        monitor_thread.daemon = True
        monitor_thread.start()
        return monitor_thread
    
    def stop_monitoring(self):
        """停止监视"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("✅ 串口连接已关闭")

def main():
    """主函数"""
    # 配置参数
    SERIAL_PORT = None  # 设为None自动查找，或指定如 'COM3' '/dev/ttyUSB0'
    BAUD_RATE = 115200
    
    print("🤖 关节串口监视器")
    print("=" * 40)
    
    # 创建监视器
    monitor = JointMonitorSerial(SERIAL_PORT, BAUD_RATE)
    
    # 如果没有指定串口，显示可用串口
    if SERIAL_PORT is None:
        available_ports = monitor.find_serial_ports()
        print(f"可用串口: {available_ports}")
        if available_ports:
            print(f"将自动连接到: {available_ports[0]}")
        else:
            print("⚠️  未找到串口，请检查ESP32连接")
    
    # 加载模型
    model = mujoco.MjModel.from_xml_path('arm1.xml')
    data = mujoco.MjData(model)
    
    # 启动监视线程
    monitor_thread = monitor.start_monitoring(model, data)
    
    try:
        # 启动MuJoCo查看器
        print("\n🎯 启动MuJoCo查看器...")
        print("操作提示:")
        print("- 鼠标左键拖拽: 旋转视角")
        print("- 鼠标滚轮: 缩放")
        print("- 鼠标右键拖拽: 平移")
        print("- ESC键: 退出查看器")
        print("- Ctrl+C: 强制退出")
        print("=" * 40)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                # 运行物理仿真
                mujoco.mj_step(model, data)
                
                # 同步查看器
                viewer.sync()
                
                # 小延迟避免CPU占用过高
                time.sleep(0.001)
                
    except KeyboardInterrupt:
        print("\n⏹️  用户中断，正在关闭...")
    except Exception as e:
        print(f"\n❌ 查看器错误: {e}")
    finally:
        # 清理资源
        monitor.stop_monitoring()
        print("✅ 程序已退出")

if __name__ == "__main__":
    main()