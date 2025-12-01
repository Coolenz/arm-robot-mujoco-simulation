#!/usr/bin/env python3
"""
简单的MuJoCo可视化界面 - arm1模型
"""
import mujoco
import mujoco.viewer
import os

# 加载模型
model = mujoco.MjModel.from_xml_path("arm1.xml")
data = mujoco.MjData(model)

# 启动查看器
print("正在启动MuJoCo查看器...")
viewer = mujoco.viewer.launch_passive(model, data)

print("")
print("模型信息:")
print("来源: arm1.xml")
print(f"关节: {model.njnt}个")
print(f"刚体: {model.nbody}个")
print(f"几何体: {model.ngeom}个")
print(f"自由度: {model.nq}")
print("")
print("操作提示:")
print("- 鼠标左键拖拽: 旋转视角")
print("- 鼠标滚轮: 缩放")
print("- 鼠标右键拖拽: 平移")
print("- ESC键: 退出查看器")

# 保持查看器运行
print("查看器已启动！使用鼠标控制视角，ESC退出")
while viewer.is_running():
    mujoco.mj_step(model, data)
    viewer.sync()

print("查看器已关闭")
