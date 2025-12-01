#!/usr/bin/env python3
"""
URDF到MJCF转换器 - 最终版本
处理STL文件路径问题，确保MuJoCo能够正确加载
"""

import os
import shutil
import mujoco
import tempfile

def convert_urdf_to_mjcf():
    """将URDF文件转换为MJCF格式"""
    
    # 定义路径
    urdf_path = '/Users/hejinglong/Documents/数字孪生机械臂/Arm_urdf2/urdf/URDF.urdf'
    meshes_dir = '/Users/hejinglong/Documents/数字孪生机械臂/Arm_urdf2/meshes'
    output_xml = '/Users/hejinglong/Documents/数字孪生机械臂/Arm_urdf2/URDF_converted.xml'
    
    print("🔄 开始URDF到MJCF转换...")
    
    # 检查文件是否存在
    if not os.path.exists(urdf_path):
        print(f"❌ 错误: 找不到URDF文件 {urdf_path}")
        return False
    
    if not os.path.exists(meshes_dir):
        print(f"❌ 错误: 找不到meshes目录 {meshes_dir}")
        return False
    
    # 使用临时目录进行转换
    with tempfile.TemporaryDirectory() as temp_dir:
        print(f"📁 创建临时工作目录: {temp_dir}")
        
        # 复制URDF文件到临时目录
        temp_urdf = os.path.join(temp_dir, 'URDF.urdf')
        
        # 读取并修复URDF内容
        with open(urdf_path, 'r', encoding='utf-8') as f:
            urdf_content = f.read()
        
        # 修复STL文件路径 - 使用文件名而不是相对路径
        # MuJoCo会在当前工作目录中查找文件
        urdf_content = urdf_content.replace('package://URDF/meshes/', '')
        urdf_content = urdf_content.replace('meshes/', '')
        
        # 保存修复后的URDF
        with open(temp_urdf, 'w', encoding='utf-8') as f:
            f.write(urdf_content)
        
        print("📋 修复URDF文件路径...")
        
        # 复制所有STL文件到临时目录（工作目录）
        stl_files = []
        for stl_file in os.listdir(meshes_dir):
            if stl_file.endswith('.STL'):
                src_path = os.path.join(meshes_dir, stl_file)
                dst_path = os.path.join(temp_dir, stl_file)
                shutil.copy2(src_path, dst_path)
                stl_files.append(stl_file)
        
        print(f"📦 复制了 {len(stl_files)} 个STL文件到工作目录")
        
        # 切换到临时目录
        original_cwd = os.getcwd()
        os.chdir(temp_dir)
        
        try:
            # 尝试加载模型
            print("🔍 正在从URDF加载模型...")
            model = mujoco.MjModel.from_xml_path(temp_urdf)
            print("✅ 模型加载成功！")
            
            # 获取模型信息
            n_joints = model.njnt
            n_bodies = model.nbody
            n_geoms = model.ngeom
            
            print(f"📊 模型信息:")
            print(f"   关节数: {n_joints}")
            print(f"   刚体数: {n_bodies}")
            print(f"   几何体数: {n_geoms}")
            
            # 保存为MJCF
            temp_output = os.path.join(temp_dir, 'converted.xml')
            mujoco.mj_saveLastXML(temp_output, model)
            print(f"💾 MJCF文件已保存到临时目录")
            
            # 将转换后的文件复制到最终位置
            shutil.copy2(temp_output, output_xml)
            print(f"✅ 转换完成！MJCF文件已保存到: {output_xml}")
            
            return True
            
        except Exception as e:
            print(f"❌ 转换过程中发生错误:")
            print(f"   {e}")
            
            # 提供调试信息
            print("\n🔍 调试信息:")
            print(f"   当前工作目录: {os.getcwd()}")
            print(f"   URDF文件存在: {os.path.exists(temp_urdf)}")
            
            # 检查STL文件
            for stl_file in ['base_link.STL', 'Empty_Link1.STL']:
                exists = os.path.exists(os.path.join(temp_dir, stl_file))
                print(f"   {stl_file} 存在: {exists}")
            
            return False
            
        finally:
            os.chdir(original_cwd)

if __name__ == "__main__":
    success = convert_urdf_to_mjcf()
    if success:
        print("\n🎉 URDF到MJCF转换成功完成！")
        print("💡 您现在可以使用以下命令查看转换后的模型:")
        print("   mjpython open_viewer.py")
    else:
        print("\n💥 URDF到MJCF转换失败，请检查错误信息。")