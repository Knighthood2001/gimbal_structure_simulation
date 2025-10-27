import rospy
import math
import argparse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState  # 用于获取关节当前状态

"""
该脚本核心逻辑是基于当前关节角度判断是否执行转动，避免无效动作。
"""

def angle_to_radian(angle_deg):
    """将角度 (度) 转换为弧度"""
    return math.radians(angle_deg)


def radian_to_angle(radian):
    """将弧度转换为角度 (度) """
    return math.degrees(radian)


def get_current_joint_angle(joint_name, timeout=5.0):
    """
    获取指定关节的当前角度 (度) 
    joint_name: 关节名称 (需与URDF中定义一致) 
    timeout: 等待关节状态消息的超时时间 (秒) 
    返回：关节当前角度 (度) , 超时或找不到关节时返回None
    """
    try:
        # 等待关节状态消息 (同步获取, 避免异步订阅的延迟问题) 
        joint_state = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
        # 查找目标关节在消息中的索引
        if joint_name in joint_state.name:
            idx = joint_state.name.index(joint_name)
            current_rad = joint_state.position[idx]  # 弧度
            return radian_to_angle(current_rad)  # 转换为度
        else:
            rospy.logerr(f"未找到关节名称: {joint_name}, 请检查URDF定义")
            return None
    except rospy.ROSException as e:
        rospy.logerr(f"获取关节状态超时 ({timeout}秒) : {str(e)}")
        return None


def publish_command(controller_topic, target_radian, duration=2.0):
    """向指定控制器发布目标角度命令, 持续duration秒"""
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz发布频率
    start_time = rospy.get_time()
    
    while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(Float64(data=target_radian))
        rate.sleep()
    rospy.loginfo(f"已向{controller_topic}发布命令{duration}秒")


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="云台关节控制脚本 (基于当前角度判断是否转动) ")
    parser.add_argument(
        "z_angle", 
        type=float, 
        nargs='?', 
        default=20.0, 
        help="Z轴 (水平旋转) 目标角度 (度) , 默认20.0度"
    )
    parser.add_argument(
        "y_angle", 
        type=float, 
        nargs='?', 
        default=30.0, 
        help="Y轴 (垂直旋转) 目标角度 (度) , 默认30.0度"
    )
    args = parser.parse_args()

    # 初始化ROS节点
    rospy.init_node('joint_sequence_controller')
    rospy.loginfo("开始获取当前关节角度...")

    # -------------------------- 关键修改：获取当前角度 --------------------------
    # 注意：关节名称需与你的URDF模型中定义的一致 (请根据实际情况修改！) 
    z_joint_name = "root_to_low_joint"  # Z轴旋转关节名称
    y_joint_name = "low_to_high_joint"  # Y轴旋转关节名称

    # 获取当前Z轴角度
    current_z_deg = get_current_joint_angle(z_joint_name)
    if current_z_deg is None:
        rospy.logerr("无法获取Z轴当前角度, 退出控制")
        return

    # 获取当前Y轴角度
    current_y_deg = get_current_joint_angle(y_joint_name)
    if current_y_deg is None:
        rospy.logerr("无法获取Y轴当前角度, 退出控制")
        return

    rospy.loginfo(f"当前角度 - Z轴: {current_z_deg:.2f}度, Y轴: {current_y_deg:.2f}度")
    # --------------------------------------------------------------------------

    # 控制Z轴 (仅当目标角度与当前角度差异超过阈值时执行) 
    z_target_deg = args.z_angle
    angle_threshold = 0.0005  # 角度误差阈值 (小于此值认为无需转动) 
    if abs(z_target_deg - current_z_deg) < angle_threshold:
        rospy.loginfo(f"Z轴当前角度({current_z_deg:.2f}度)已接近目标({z_target_deg}度), 无需转动")
    else:
        rospy.loginfo(f"开始控制Z轴: 从{current_z_deg:.2f}度转动到{z_target_deg}度")
        z_target_rad = angle_to_radian(z_target_deg)
        publish_command(
            controller_topic="/root_to_low_position_controller/command",
            target_radian=z_target_rad,
            duration=3.0
        )
        rospy.sleep(1.0)  # 等待Z轴转动完成

    # 控制Y轴 (同理判断) 
    y_target_deg = args.y_angle
    if abs(y_target_deg - current_y_deg) < angle_threshold:
        rospy.loginfo(f"Y轴当前角度({current_y_deg:.2f}度)已接近目标({y_target_deg}度), 无需转动")
    else:
        rospy.loginfo(f"开始控制Y轴: 从{current_y_deg:.2f}度转动到{y_target_deg}度")
        y_target_rad = angle_to_radian(y_target_deg)
        publish_command(
            controller_topic="/low_to_high_position_controller/command",
            target_radian=y_target_rad,
            duration=3.0
        )

    rospy.loginfo("所有控制逻辑执行完成")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")