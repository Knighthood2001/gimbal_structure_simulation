#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <cmath>       // 用于角度转弧度（M_PI）
#include <string>      // 用于字符串处理
#include <stdexcept>   // 用于处理参数转换异常（如非数字输入）

/**
 * 功能：将角度（度）转换为弧度
 * @param angle_deg 输入角度（度）
 * @return 对应的弧度值
 */
double angleToRadian(double angle_deg) {
    return angle_deg * M_PI / 180.0;  // 等价于Python的math.radians
}

/**
 * 功能：向指定ROS话题发布目标弧度命令，持续指定时间
 * @param controller_topic 控制器话题名称（如"/root_to_low_position_controller/command"）
 * @param target_radian 目标位置（弧度）
 * @param duration 发布持续时间（秒），确保关节有足够时间到达目标
 */
void publishCommand(const std::string& controller_topic, double target_radian, double duration) {
    ros::NodeHandle nh;  // 创建节点句柄（C++中发布者需绑定句柄）
    // 初始化发布者：话题名、消息类型、队列大小10
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(controller_topic, 10);

    // 等待发布者与订阅者建立连接（最多等待2秒，避免启动时消息丢失）
    int wait_count = 0;
    while (pub.getNumSubscribers() == 0 && ros::ok() && wait_count < 20) {
        if (wait_count == 0) {
            ROS_WARN_STREAM("等待订阅者连接话题：" << controller_topic);
        }
        ros::Duration(0.1).sleep();  // 每0.1秒检查一次
        wait_count++;
    }
    if (pub.getNumSubscribers() == 0) {
        ROS_WARN_STREAM("话题 " << controller_topic << " 无订阅者，可能无法控制关节");
    }

    // 配置发布频率（10Hz，与Python版本一致）
    ros::Rate rate(10);
    ros::Time start_time = ros::Time::now();  // 记录发布开始时间
    std_msgs::Float64 msg;
    msg.data = target_radian;  // 设置目标弧度值

    // 持续发布指定时间，或节点关闭时退出
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < duration) {
        pub.publish(msg);
        rate.sleep();  // 保持10Hz发布频率
    }

    ROS_INFO_STREAM("已向话题 [" << controller_topic << "] 发布命令 " << duration << " 秒");
}

/**
 * 功能：打印脚本使用帮助信息（当参数输入错误时调用）
 * @param prog_name 程序名称（argv[0]）
 */
void printUsage(const char* prog_name) {
    ROS_INFO("\n脚本用法：%s [z_angle] [y_angle]", prog_name);
    ROS_INFO("  功能：控制云台绕Z轴（水平）和Y轴（垂直）旋转，支持命令行输入目标角度（度）");
    ROS_INFO("  参数说明：");
    ROS_INFO("    z_angle  - Z轴（水平旋转）目标角度（度），可选，默认20.0度");
    ROS_INFO("    y_angle  - Y轴（垂直旋转）目标角度（度），可选，默认30.0度");
    ROS_INFO("  示例：");
    ROS_INFO("    1. 使用默认角度：%s", prog_name);
    ROS_INFO("    2. 自定义角度（Z=10度，Y=15度）：%s 10 15", prog_name);
    ROS_INFO("    3. 单参数（Z=-5度，Y用默认）：%s -5\n", prog_name);
}

int main(int argc, char** argv) {
    // -------------------------- 1. 解析命令行参数 --------------------------
    double z_target_deg = 20.0;  // Z轴默认角度（与Python一致）
    double y_target_deg = 30.0;  // Y轴默认角度（与Python一致）

    try {
        // 根据参数个数（argc）分配角度值：argc=1→默认值；argc>=2→Z轴用argv[1]；argc>=3→Y轴用argv[2]
        if (argc >= 2) {
            // 将字符串参数（argv[1]）转换为double，若转换失败会抛invalid_argument异常
            z_target_deg = std::stod(argv[1]);
        }
        if (argc >= 3) {
            y_target_deg = std::stod(argv[2]);
        }
        // 若参数过多（argc>3），提示冗余参数
        if (argc > 3) {
            ROS_WARN_STREAM("参数过多：仅需Z轴和Y轴角度，多余参数已忽略");
        }
    } 
    // 捕获参数转换异常（如输入非数字："abc"、"12a"等）
    catch (const std::invalid_argument& e) {
        ROS_ERROR_STREAM("参数错误：输入的角度不是有效数字！错误信息：" << e.what());
        printUsage(argv[0]);  // 打印帮助信息
        return 1;  // 异常退出，返回错误码1
    }
    // 捕获数值超出范围异常（如输入极大/极小值）
    catch (const std::out_of_range& e) {
        ROS_ERROR_STREAM("参数错误：角度值超出范围！错误信息：" << e.what());
        printUsage(argv[0]);
        return 1;
    }

    // -------------------------- 2. 初始化ROS节点 --------------------------
    ros::init(argc, argv, "joint_sequence_controller");  // 节点名与Python一致
    ros::NodeHandle nh;  // 全局节点句柄（用于ROS通信）
    ROS_INFO("云台控制节点启动成功！");
    ROS_INFO("当前目标角度：Z轴=%.1f度，Y轴=%.1f度", z_target_deg, y_target_deg);

    // -------------------------- 3. 控制Z轴旋转 --------------------------
    double z_target_rad = angleToRadian(z_target_deg);
    ROS_INFO_STREAM("\n开始控制Z轴（水平旋转）：" << z_target_deg << "度 → " << z_target_rad << "弧度");
    publishCommand(
        "/root_to_low_position_controller/command",  // Z轴控制器话题
        z_target_rad,                                // 目标弧度
        3.0                                          // 发布持续时间（3秒）
    );

    // 等待1秒，确保Z轴运动稳定后再控制Y轴（与Python逻辑一致）
    ros::Duration(1.0).sleep();

    // -------------------------- 4. 控制Y轴旋转 --------------------------
    double y_target_rad = angleToRadian(y_target_deg);
    ROS_INFO_STREAM("\n开始控制Y轴（垂直旋转）：" << y_target_deg << "度 → " << y_target_rad << "弧度");
    publishCommand(
        "/low_to_high_position_controller/command",  // Y轴控制器话题
        y_target_rad,                                // 目标弧度
        3.0                                          // 发布持续时间（3秒）
    );

    // -------------------------- 5. 任务完成 --------------------------
    ROS_INFO("\n所有控制命令发布完成！");

    // C++中无需显式spin（发布逻辑已在循环中完成），直接退出即可
    return 0;
}