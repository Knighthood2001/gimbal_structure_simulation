import rospy
import math
import time
from std_msgs.msg import Float64

def angle_to_radian(angle_deg):
    """将角度（度）转换为弧度"""
    return math.radians(angle_deg)

def publish_command(controller_topic, target_radian, duration=2.0):
    """
    向指定控制器发布目标角度命令，持续duration秒后退出
    controller_topic: 控制器话题
    target_radian: 目标弧度
    duration: 发布持续时间（确保关节有足够时间到达目标位置）
    """
    pub = rospy.Publisher(controller_topic, Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    start_time = rospy.get_time()
    
    # 持续发布duration秒
    while rospy.get_time() - start_time < duration and not rospy.is_shutdown():
        pub.publish(Float64(data=target_radian))
        rate.sleep()
    rospy.loginfo(f"已向{controller_topic}发布命令{duration}秒")

def main():
    # 只初始化一次节点
    rospy.init_node('joint_sequence_controller')
    
    # 1. 控制Z轴旋转（root_to_low_joint）
    z_target_deg = 0.0  # 目标角度（度）
    z_target_rad = angle_to_radian(z_target_deg)
    rospy.loginfo(f"开始控制Z轴：{z_target_deg}度")
    publish_command(
        controller_topic="/root_to_low_position_controller/command",
        target_radian=z_target_rad,
        duration=3.0  # 发布3秒，确保关节转到目标位置
    )
    # 等待1秒（可选）
    rospy.sleep(1.0)
    # 2. 控制Y轴旋转（low_to_high_joint）
    y_target_deg = 0.0  # 目标角度（度）
    y_target_rad = angle_to_radian(y_target_deg)
    rospy.loginfo(f"开始控制Y轴：{y_target_deg}度")
    publish_command(
        controller_topic="/low_to_high_position_controller/command",
        target_radian=y_target_rad,
        duration=3.0
    )
    
    rospy.loginfo("所有控制命令发布完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")