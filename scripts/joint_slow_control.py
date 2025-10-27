import rospy
import math
import argparse
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
"""
该脚本用于控制云台结构的 Z 轴(水平旋转)和 Y 轴(垂直旋转)关节，实现缓慢且可控速度的角度调整。核心功能如下:
1. 支持通过命令行输入目标角度(Z 轴和 Y 轴)及移动速度(Z 轴和 Y 轴)，未输入时使用默认值(默认角度: Z 轴 20 度、Y 轴 30 度；默认速度: 10 度 / 秒)。
2. 自动获取当前关节角度(通过 ROS 的/joint_states话题)，仅当目标角度与当前角度差异超过 0.05 度时才执行移动，避免无效动作。
3. 采用轨迹规划实现匀速缓慢移动: 根据角度差和指定速度计算总移动时间，通过线性插值生成中间位置并平滑发布，确保运动平稳。
4. 移动完成后会补充发布一次目标角度，保证关节精确到达指定位置。
"""

def angle_to_radian(angle_deg):
    """角度转弧度"""
    return math.radians(angle_deg)


def radian_to_angle(radian):
    """弧度转角度"""
    return math.degrees(radian)


def get_current_joint_angle(joint_name, timeout=5.0):
    """获取指定关节当前角度(度)"""
    try:
        joint_state = rospy.wait_for_message('/joint_states', JointState, timeout=timeout)
        if joint_name in joint_state.name:
            idx = joint_state.name.index(joint_name)
            return radian_to_angle(joint_state.position[idx])
        else:
            rospy.logerr(f"未找到关节: {joint_name}(请检查URDF)")
            return None
    except rospy.ROSException:
        rospy.logerr(f"获取关节状态超时({timeout}秒)")
        return None


def publish_slow_movement(controller_topic, start_deg, target_deg, speed_deg_per_sec, rate_hz=20):
    """
    匀速缓慢移动到目标角度(位置控制轨迹规划)
    controller_topic: 控制器话题
    start_deg: 起始角度(度)
    target_deg: 目标角度(度)
    speed_deg_per_sec: 移动速度(度/秒，正数)
    rate_hz: 发布频率(Hz，越高越平滑)
    """
    # 计算角度差和总移动时间
    angle_diff = target_deg - start_deg
    total_angle = abs(angle_diff)
    
    # 角度差过小，无需移动
    if total_angle < 0.1:  # 阈值0.1度
        rospy.loginfo("角度差过小，无需移动")
        return
    
    # 计算总移动时间(秒)
    total_time = total_angle / speed_deg_per_sec
    rospy.loginfo(f"开始移动: 从{start_deg:.2f}度到{target_deg:.2f}度，速度{speed_deg_per_sec}度/秒，预计{total_time:.2f}秒")
    
    # 初始化发布器和时间控制
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    rate = rospy.Rate(rate_hz)
    start_time = rospy.get_time()
    current_time = 0.0
    
    # 线性插值发布中间位置
    while current_time < total_time and not rospy.is_shutdown():
        # 计算当前时间在总时间中的占比(0~1)
        progress = current_time / total_time
        # 线性插值当前角度(度)
        current_deg = start_deg + progress * angle_diff
        # 转换为弧度并发布
        pub.publish(Float64(data=angle_to_radian(current_deg)))
        # 等待下一个周期
        rate.sleep()
        # 更新当前时间
        current_time = rospy.get_time() - start_time
    
    # 最后发布一次目标角度(确保精确到达)
    pub.publish(Float64(data=angle_to_radian(target_deg)))
    rospy.loginfo(f"移动完成，已到达目标角度")


def main():
    # 解析命令行参数(新增速度参数)
    parser = argparse.ArgumentParser(description="云台关节控制(支持指定速度缓慢移动)")
    parser.add_argument("z_angle", type=float, nargs='?', default=20.0, help="Z轴目标角度(度)，默认20度")
    parser.add_argument("y_angle", type=float, nargs='?', default=30.0, help="Y轴目标角度(度)，默认30度")
    parser.add_argument("z_speed", type=float, nargs='?', default=10.0, help="Z轴移动速度(度/秒)，默认10度/秒")
    parser.add_argument("y_speed", type=float, nargs='?', default=10.0, help="Y轴移动速度(度/秒)，默认10度/秒")
    args = parser.parse_args()

    # 初始化节点
    rospy.init_node('joint_slow_controller')
    rospy.loginfo("开始缓慢移动控制...")

    # 关节名称(需与URDF一致，务必修改！)
    z_joint_name = "root_to_low_joint"  # 替换为实际Z轴关节名
    y_joint_name = "low_to_high_joint"  # 替换为实际Y轴关节名

    # 获取当前角度
    current_z = get_current_joint_angle(z_joint_name)
    current_y = get_current_joint_angle(y_joint_name)
    if current_z is None or current_y is None:
        rospy.logerr("无法获取当前角度，退出")
        return

    rospy.loginfo(f"当前角度 - Z: {current_z:.2f}度, Y: {current_y:.2f}度")
    rospy.loginfo(f"目标角度 - Z: {args.z_angle}度, Y: {args.y_angle}度")

    # 控制Z轴缓慢移动
    if abs(args.z_angle - current_z) > 0.05:  # 角度差>0.05度才移动
        publish_slow_movement(
            controller_topic="/root_to_low_position_controller/command",
            start_deg=current_z,
            target_deg=args.z_angle,
            speed_deg_per_sec=args.z_speed  # 用户指定的Z轴速度
        )
    else:
        rospy.loginfo("Z轴角度已达标，无需移动")

    # 等待Z轴完成(可选，根据机械结构是否需要先后顺序)
    rospy.sleep(0.5)

    # 控制Y轴缓慢移动
    if abs(args.y_angle - current_y) > 0.05:
        publish_slow_movement(
            controller_topic="/low_to_high_position_controller/command",
            start_deg=current_y,
            target_deg=args.y_angle,
            speed_deg_per_sec=args.y_speed  # 用户指定的Y轴速度
        )
    else:
        rospy.loginfo("Y轴角度已达标，无需移动")

    rospy.loginfo("所有移动控制完成")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误: {str(e)}")