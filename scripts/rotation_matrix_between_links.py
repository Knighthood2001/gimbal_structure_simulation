#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf.transformations as tft  # 用于四元数转旋转矩阵
"""
功能: 获取source_link到target_link的旋转变换矩阵
"""
class LinkTransformDemo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('link_transform_demo', anonymous=True)
        
        # 初始化TF缓冲区和监听器（用于监听/tf话题）
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 定义要查询的两个link的坐标系名称（替换为你的link名称）
        self.source_link = "velodyne1"    # 源link
        self.target_link = "high_link"     # 目标link
        
        # 等待TF变换发布（给系统一点时间初始化）
        rospy.sleep(1.0)
        
        # 获取并打印旋转变换矩阵
        self.get_rotation_matrix()

    def get_rotation_matrix(self):
        """获取source_link到target_link的旋转变换矩阵"""
        try:
            # 查找两个link之间的变换（target_link为参考系，source_link相对于它的变换）
            # rospy.Time(0)表示获取最新的变换，超时5秒
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.target_link,  # 目标坐标系（参考系）
                source_frame=self.source_link,  # 源坐标系（需要转换的link）
                time=rospy.Time(0),
                timeout=rospy.Duration(5.0)
            )
        except (tf2_ros.LookupException,    # 找不到变换
                tf2_ros.ConnectivityException,  # 连接问题
                tf2_ros.ExtrapolationException) as e:  # 时间戳问题
            rospy.logerr(f"获取TF变换失败: {str(e)}")
            return

        # 提取旋转四元数（x, y, z, w）
        quat = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ]

        # 提取平移分量（x, y, z）
        translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]

        # 1. 转换为3x3纯旋转变换矩阵
        rotation_matrix_3x3 = tft.quaternion_matrix(quat)[:3, :3]  # 取4x4矩阵的前3行3列

        # 2. 转换为4x4齐次变换矩阵（包含旋转和平移）
        homogeneous_matrix_4x4 = tft.quaternion_matrix(quat)  # 先构造旋转部分的4x4矩阵
        homogeneous_matrix_4x4[:3, 3] = translation  # 填充平移分量

        # 打印结果
        print(f"\n{self.source_link} 相对于 {self.target_link} 的变换矩阵:")
        print("3x3纯旋转变换矩阵:\n", rotation_matrix_3x3)
        print("\n4x4齐次变换矩阵（旋转+平移）:\n", homogeneous_matrix_4x4)

if __name__ == "__main__":
    try:
        LinkTransformDemo()
    except rospy.ROSInterruptException:
        pass