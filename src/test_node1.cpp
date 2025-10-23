/**
 * @file test_node1.cpp
 * @brief 测试节点1: 验证MAVROS连接、解锁、起飞3m、悬停30s、自动降落
 * 
 * 功能说明：
 * 1. 等待与MAVROS的连接建立
 * 2. 切换到OFFBOARD模式
 * 3. 解锁无人机
 * 4. 起飞到3m高度
 * 5. 在3m高度悬停30秒
 * 6. 切换到AUTO.LAND模式自动降落
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TestNode1 {
public:
    TestNode1() : nh_(""), nh_private_("~") {
        // 初始化参数
        nh_private_.param<double>("target_height", target_height_, 3.0);
        nh_private_.param<double>("hover_duration", hover_duration_, 30.0);
        nh_private_.param<double>("position_tolerance", position_tolerance_, 0.2);
        
        // 获取mavros命名空间前缀（如果有的话）
        std::string mavros_ns;
        nh_private_.param<std::string>("mavros_ns", mavros_ns, "/mavros");
        
        ROS_INFO("Using MAVROS namespace: %s", mavros_ns.c_str());
        
        // 订阅器
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            mavros_ns + "/state", 10, &TestNode1::stateCallback, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            mavros_ns + "/local_position/pose", 10, &TestNode1::poseCallback, this);
        
        // 发布器
        local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
            mavros_ns + "/setpoint_position/local", 10);
        
        // 服务客户端
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            mavros_ns + "/set_mode");
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            mavros_ns + "/cmd/arming");
        
        ROS_INFO("TestNode1 initialized");
        ROS_INFO("Target height: %.2f m", target_height_);
        ROS_INFO("Hover duration: %.2f s", hover_duration_);
    }
    
    void run() {
        ros::Rate rate(20.0);  // 20Hz控制频率
        
        // 阶段1：等待MAVROS连接
        ROS_INFO("=== Stage 1: Waiting for MAVROS connection ===");
        if (!waitForConnection(30.0)) {
            ROS_ERROR("Failed to connect to MAVROS. Exiting.");
            return;
        }
        ROS_INFO("MAVROS connected successfully!");
        
        // 阶段2：发送初始位置setpoint（OFFBOARD模式前必须发送）
        ROS_INFO("=== Stage 2: Sending initial setpoints ===");
        geometry_msgs::PoseStamped target_pose = createTargetPose(0, 0, target_height_);
        
        // 在切换到OFFBOARD前，至少发送100个setpoint
        for(int i = 0; i < 100; ++i) {
            local_pos_pub_.publish(target_pose);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Initial setpoints sent");
        
        // 阶段3：切换到OFFBOARD模式
        ROS_INFO("=== Stage 3: Switching to OFFBOARD mode ===");
        if (!setMode("OFFBOARD", 10.0)) {
            ROS_ERROR("Failed to set OFFBOARD mode. Exiting.");
            return;
        }
        ROS_INFO("OFFBOARD mode set successfully!");
        
        // 阶段4：解锁
        ROS_INFO("=== Stage 4: Arming ===");
        if (!arm(10.0)) {
            ROS_ERROR("Failed to arm. Exiting.");
            return;
        }
        ROS_INFO("Armed successfully!");
        
        // 阶段5：起飞到目标高度
        ROS_INFO("=== Stage 5: Taking off to %.2f m ===", target_height_);
        ros::Time takeoff_start = ros::Time::now();
        bool reached_height = false;
        
        while (ros::ok() && !reached_height) {
            // 持续发布目标位置
            local_pos_pub_.publish(target_pose);
            
            // 检查是否到达目标高度
            if (hasReachedPosition(target_pose)) {
                reached_height = true;
                ROS_INFO("Reached target height: %.2f m", current_pose_.pose.position.z);
            }
            
            // 超时检查（60秒）
            if ((ros::Time::now() - takeoff_start).toSec() > 60.0) {
                ROS_WARN("Takeoff timeout. Current height: %.2f m", 
                         current_pose_.pose.position.z);
                break;
            }
            
            // 每5秒输出一次当前高度
            static ros::Time last_print = ros::Time::now();
            if ((ros::Time::now() - last_print).toSec() > 5.0) {
                ROS_INFO("Current height: %.2f m (target: %.2f m)", 
                         current_pose_.pose.position.z, target_height_);
                last_print = ros::Time::now();
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        if (!reached_height) {
            ROS_ERROR("Failed to reach target height. Landing...");
            land();
            return;
        }
        
        // 阶段6：悬停
        ROS_INFO("=== Stage 6: Hovering for %.2f seconds ===", hover_duration_);
        ros::Time hover_start = ros::Time::now();
        
        while (ros::ok() && (ros::Time::now() - hover_start).toSec() < hover_duration_) {
            // 持续发布目标位置以保持悬停
            local_pos_pub_.publish(target_pose);
            
            // 每5秒输出一次剩余时间
            static ros::Time last_hover_print = ros::Time::now();
            if ((ros::Time::now() - last_hover_print).toSec() > 5.0) {
                double remaining = hover_duration_ - (ros::Time::now() - hover_start).toSec();
                ROS_INFO("Hovering... %.1f seconds remaining", remaining);
                last_hover_print = ros::Time::now();
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("Hover complete!");
        
        // 阶段7：自动降落
        ROS_INFO("=== Stage 7: Landing (AUTO.LAND mode) ===");
        if (!land()) {
            ROS_ERROR("Failed to initiate landing");
        } else {
            ROS_INFO("Landing mode activated. Waiting for landing to complete...");
            
            // 等待降落完成（检测高度接近0且上锁）
            ros::Time land_start = ros::Time::now();
            while (ros::ok()) {
                // 检查是否已经降落并上锁
                if (!current_state_.armed && current_pose_.pose.position.z < 0.1) {
                    ROS_INFO("Landing complete! Disarmed and on ground.");
                    break;
                }
                
                // 超时检查（2分钟）
                if ((ros::Time::now() - land_start).toSec() > 120.0) {
                    ROS_WARN("Landing timeout. Current state: armed=%d, height=%.2f m",
                             current_state_.armed, current_pose_.pose.position.z);
                    break;
                }
                
                // 每10秒输出一次状态
                static ros::Time last_land_print = ros::Time::now();
                if ((ros::Time::now() - last_land_print).toSec() > 10.0) {
                    ROS_INFO("Landing... height: %.2f m, armed: %d", 
                             current_pose_.pose.position.z, current_state_.armed);
                    last_land_print = ros::Time::now();
                }
                
                ros::spinOnce();
                rate.sleep();
            }
        }
        
        ROS_INFO("=== Test Complete ===");
    }

private:
    // ROS节点句柄
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // 订阅器和发布器
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_;
    ros::Publisher local_pos_pub_;
    
    // 服务客户端
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    // 状态变量
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    bool pose_received_ = false;
    
    // 参数
    double target_height_;
    double hover_duration_;
    double position_tolerance_;
    
    /**
     * @brief 状态回调函数
     */
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }
    
    /**
     * @brief 位置回调函数
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }
    
    /**
     * @brief 创建目标位置
     */
    geometry_msgs::PoseStamped createTargetPose(double x, double y, double z) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        
        // 设置朝向为默认（朝向北方）
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        pose.pose.orientation = tf2::toMsg(q);
        
        return pose;
    }
    
    /**
     * @brief 等待MAVROS连接
     * @param timeout 超时时间（秒）
     * @return 是否成功连接
     */
    bool waitForConnection(double timeout) {
        ros::Rate rate(10.0);
        ros::Time start = ros::Time::now();
        
        while (ros::ok()) {
            if (current_state_.connected) {
                return true;
            }
            
            if ((ros::Time::now() - start).toSec() > timeout) {
                return false;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        return false;
    }
    
    /**
     * @brief 设置飞行模式
     * @param mode 目标模式
     * @param timeout 超时时间（秒）
     * @return 是否成功
     */
    bool setMode(const std::string& mode, double timeout) {
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.custom_mode = mode;
        
        ros::Rate rate(5.0);
        ros::Time start = ros::Time::now();
        
        while (ros::ok()) {
            // 检查是否已经是目标模式
            if (current_state_.mode == mode) {
                return true;
            }
            
            // 调用服务
            if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent) {
                ROS_INFO("Mode change request sent: %s", mode.c_str());
            }
            
            // 超时检查
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_ERROR("Set mode timeout. Current mode: %s", current_state_.mode.c_str());
                return false;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        return false;
    }
    
    /**
     * @brief 解锁无人机
     * @param timeout 超时时间（秒）
     * @return 是否成功
     */
    bool arm(double timeout) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        
        ros::Rate rate(5.0);
        ros::Time start = ros::Time::now();
        
        while (ros::ok()) {
            // 检查是否已经解锁
            if (current_state_.armed) {
                return true;
            }
            
            // 调用服务
            if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Arm command sent");
            }
            
            // 超时检查
            if ((ros::Time::now() - start).toSec() > timeout) {
                ROS_ERROR("Arming timeout");
                return false;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
        
        return false;
    }
    
    /**
     * @brief 切换到降落模式
     * @return 是否成功发送降落命令
     */
    bool land() {
        return setMode("AUTO.LAND", 5.0);
    }
    
    /**
     * @brief 检查是否到达目标位置
     * @param target 目标位置
     * @return 是否到达
     */
    bool hasReachedPosition(const geometry_msgs::PoseStamped& target) {
        if (!pose_received_) {
            return false;
        }
        
        double dx = current_pose_.pose.position.x - target.pose.position.x;
        double dy = current_pose_.pose.position.y - target.pose.position.y;
        double dz = current_pose_.pose.position.z - target.pose.position.z;
        
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        return distance < position_tolerance_;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node1");
    
    ROS_INFO("========================================");
    ROS_INFO("  Test Node 1: Basic Flight Test");
    ROS_INFO("========================================");
    ROS_INFO("This node will:");
    ROS_INFO("  1. Connect to MAVROS");
    ROS_INFO("  2. Arm the vehicle");
    ROS_INFO("  3. Take off to 3m");
    ROS_INFO("  4. Hover for 30 seconds");
    ROS_INFO("  5. Auto land");
    ROS_INFO("========================================");
    
    TestNode1 node;
    node.run();
    
    return 0;
}