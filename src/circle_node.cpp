#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <limits>

class CircleFlightTest {
public:
    CircleFlightTest();
    void run();

private:
    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // 状态机
    enum FlightState {
        IDLE,
        TAKEOFF,
        MOVE_TO_CIRCLE_START,
        CIRCLE_FLIGHT,
        RETURN_HOME,
        LANDING,
        MISSION_COMPLETE
    };
    
    // 辅助函数
    void generateCircleWaypoints();
    size_t findNearestWaypoint();
    void reorderWaypointsFromIndex(size_t start_idx);
    geometry_msgs::PoseStamped createWaypoint(double x, double y, double z, double yaw);
    double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    double getYaw(const geometry_msgs::Quaternion& q);
    bool waitForConnection();
    bool setOffboardMode();
    bool armVehicle();
    
    // ROS接口
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    
    // 状态变量
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped home_pose_;
    FlightState flight_state_;
    
    // 航点
    std::vector<geometry_msgs::PoseStamped> circle_waypoints_;
    size_t current_waypoint_idx_;
    
    // 参数
    double takeoff_height_;      // 起飞高度：5m
    double circle_radius_;       // 圆圈半径：5m
    double flight_speed_;        // 飞行速度：0.5m/s
    double waypoint_tolerance_;  // 航点到达容差
    int num_waypoints_;          // 圆周航点数量
    
    // 圆心位置（相对于起飞点）
    double circle_center_x_;
    double circle_center_y_;
    
    bool pose_received_;
    bool home_set_;
    ros::Time last_request_;
};

CircleFlightTest::CircleFlightTest() : 
    nh_(""),
    flight_state_(IDLE),
    current_waypoint_idx_(0),
    pose_received_(false),
    home_set_(false)
{
    ros::NodeHandle nh_private("~");
    
    // 加载参数
    nh_private.param("takeoff_height", takeoff_height_, 5.0);
    nh_private.param("circle_radius", circle_radius_, 5.0);
    nh_private.param("flight_speed", flight_speed_, 0.5);
    nh_private.param("waypoint_tolerance", waypoint_tolerance_, 0.3);
    nh_private.param("num_waypoints", num_waypoints_, 36);  // 每10度一个航点
    nh_private.param("circle_center_x", circle_center_x_, 5.0);  // 圆心在起飞点前方5m
    nh_private.param("circle_center_y", circle_center_y_, 0.0);
    
    // 获取mavros命名空间前缀（与测试节点1保持一致）
    std::string mavros_ns;
    nh_private.param<std::string>("mavros_ns", mavros_ns, "/mavros");
    
    ROS_INFO("Using MAVROS namespace: %s", mavros_ns.c_str());
    
    // 订阅者
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        mavros_ns + "/state", 10, &CircleFlightTest::stateCallback, this);
    pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        mavros_ns + "/local_position/pose", 10, &CircleFlightTest::poseCallback, this);
    
    // 发布者
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        mavros_ns + "/setpoint_position/local", 10);
    
    // 服务客户端
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        mavros_ns + "/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        mavros_ns + "/cmd/arming");
    
    ROS_INFO("Circle Flight Test Node Initialized");
    ROS_INFO("Parameters:");
    ROS_INFO("  Takeoff Height: %.2f m", takeoff_height_);
    ROS_INFO("  Circle Radius: %.2f m", circle_radius_);
    ROS_INFO("  Flight Speed: %.2f m/s", flight_speed_);
    ROS_INFO("  Circle Center: (%.2f, %.2f)", circle_center_x_, circle_center_y_);
}

void CircleFlightTest::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void CircleFlightTest::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
    pose_received_ = true;
    
    if (!home_set_) {
        home_pose_ = current_pose_;
        home_set_ = true;
        ROS_INFO("Home position set: (%.2f, %.2f, %.2f)",
                 home_pose_.pose.position.x,
                 home_pose_.pose.position.y,
                 home_pose_.pose.position.z);
    }
}

void CircleFlightTest::generateCircleWaypoints() {
    circle_waypoints_.clear();
    
    // 获取无人机起飞时的朝向（yaw角）
    double home_yaw = getYaw(home_pose_.pose.orientation);
    
    // 根据无人机的初始朝向计算圆心位置
    // circle_center_x_ 是相对于机头方向的前方距离
    // circle_center_y_ 是相对于机头方向的左侧距离（右手坐标系）
    double center_x = home_pose_.pose.position.x 
                    + circle_center_x_ * cos(home_yaw) 
                    - circle_center_y_ * sin(home_yaw);
    double center_y = home_pose_.pose.position.y 
                    + circle_center_x_ * sin(home_yaw) 
                    + circle_center_y_ * cos(home_yaw);
    double center_z = home_pose_.pose.position.z + takeoff_height_;
    
    ROS_INFO("Home yaw angle: %.2f degrees", home_yaw * 180.0 / M_PI);
    ROS_INFO("Circle center (relative to drone): forward=%.2f m, left=%.2f m", 
             circle_center_x_, circle_center_y_);
    ROS_INFO("Circle center (world coordinates): (%.2f, %.2f, %.2f)", 
             center_x, center_y, center_z);
    
    // 生成圆周上的航点（顺时针方向）
    // 从俯视图看，角度递减表示顺时针旋转
    for (int i = 0; i <= num_waypoints_; i++) {
        // 负号表示顺时针方向（相对于地面机头朝向）
        double angle = -2.0 * M_PI * i / num_waypoints_;
        
        // 圆周上的位置
        double x = center_x + circle_radius_ * cos(angle);
        double y = center_y + circle_radius_ * sin(angle);
        double z = center_z;
        
        // 计算指向圆心的yaw角（机头指向圆心）
        double yaw = atan2(center_y - y, center_x - x);
        
        geometry_msgs::PoseStamped waypoint = createWaypoint(x, y, z, yaw);
        circle_waypoints_.push_back(waypoint);
    }
    
    ROS_INFO("Generated %lu circle waypoints (clockwise direction)", circle_waypoints_.size());
}

size_t CircleFlightTest::findNearestWaypoint() {
    if (circle_waypoints_.empty()) {
        return 0;
    }
    
    size_t nearest_idx = 0;
    double min_distance = std::numeric_limits<double>::max();
    
    // 不包括最后一个点（因为它和第一个点重合）
    for (size_t i = 0; i < circle_waypoints_.size() - 1; i++) {
        double dist = calculateDistance(current_pose_.pose.position, 
                                       circle_waypoints_[i].pose.position);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_idx = i;
        }
    }
    
    ROS_INFO("Nearest waypoint: index %lu, distance %.2f m", nearest_idx, min_distance);
    
    // 输出最近航点的位置信息
    ROS_INFO("  Current position: (%.2f, %.2f, %.2f)", 
             current_pose_.pose.position.x,
             current_pose_.pose.position.y,
             current_pose_.pose.position.z);
    ROS_INFO("  Nearest waypoint: (%.2f, %.2f, %.2f)",
             circle_waypoints_[nearest_idx].pose.position.x,
             circle_waypoints_[nearest_idx].pose.position.y,
             circle_waypoints_[nearest_idx].pose.position.z);
    
    return nearest_idx;
}

void CircleFlightTest::reorderWaypointsFromIndex(size_t start_idx) {
    if (circle_waypoints_.empty() || start_idx == 0) {
        return;
    }
    
    std::vector<geometry_msgs::PoseStamped> reordered;
    
    // 从start_idx开始到结束（不包括最后一个重复点）
    for (size_t i = start_idx; i < circle_waypoints_.size() - 1; i++) {
        reordered.push_back(circle_waypoints_[i]);
    }
    
    // 从开始到start_idx
    for (size_t i = 0; i <= start_idx; i++) {
        reordered.push_back(circle_waypoints_[i]);
    }
    
    circle_waypoints_ = reordered;
    
    ROS_INFO("Reordered waypoints to start from index %lu", start_idx);
    ROS_INFO("New sequence has %lu waypoints", circle_waypoints_.size());
}

geometry_msgs::PoseStamped CircleFlightTest::createWaypoint(double x, double y, double z, double yaw) {
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = "world";
    waypoint.header.stamp = ros::Time::now();
    
    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.position.z = z;
    
    // 将yaw角转换为四元数
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    waypoint.pose.orientation = tf2::toMsg(q);
    
    return waypoint;
}

double CircleFlightTest::calculateDistance(const geometry_msgs::Point& p1, 
                                           const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double CircleFlightTest::getYaw(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
}

bool CircleFlightTest::waitForConnection() {
    ROS_INFO("Waiting for FCU connection...");
    ros::Rate rate(20);
    while (ros::ok() && !current_state_.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected!");
    return true;
}

bool CircleFlightTest::setOffboardMode() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
        return true;
    }
    return false;
}

bool CircleFlightTest::armVehicle() {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
        return true;
    }
    return false;
}

void CircleFlightTest::run() {
    ros::Rate rate(20.0);  // 20Hz
    
    // 等待连接
    if (!waitForConnection()) {
        ROS_ERROR("Failed to connect to FCU");
        return;
    }
    
    // 等待位置信息和EKF2初始化
    ROS_INFO("Waiting for position data and EKF2 initialization...");
    ros::Time ekf_wait_start = ros::Time::now();
    bool ekf_ready = false;
    
    while (ros::ok() && !ekf_ready) {
        ros::spinOnce();
        
        // 检查是否收到位置数据
        if (pose_received_) {
            // 检查高度是否合理（应该接近0，允许±2米的误差）
            double current_height = current_pose_.pose.position.z;
            if (fabs(current_height) < 2.0) {
                ekf_ready = true;
                ROS_INFO("EKF2 initialized! Current position: (%.2f, %.2f, %.2f)", 
                         current_pose_.pose.position.x,
                         current_pose_.pose.position.y,
                         current_pose_.pose.position.z);
            } else {
                // 每5秒打印一次当前高度
                static ros::Time last_warn = ros::Time::now();
                if ((ros::Time::now() - last_warn).toSec() > 5.0) {
                    ROS_WARN("Waiting for EKF2... Current height: %.2f m (should be near 0)", 
                             current_height);
                    last_warn = ros::Time::now();
                }
            }
        }
        
        // 超时检查（30秒）
        if ((ros::Time::now() - ekf_wait_start).toSec() > 30.0) {
            ROS_ERROR("EKF2 initialization timeout! Current height: %.2f m", 
                     current_pose_.pose.position.z);
            ROS_ERROR("Please check:");
            ROS_ERROR("  1. Is the simulation running properly?");
            ROS_ERROR("  2. Is MAVROS receiving sensor data?");
            ROS_ERROR("  3. Try increasing the launch delay time.");
            return;
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Position system ready!");
    
    // 发送初始设定点（当前位置）
    geometry_msgs::PoseStamped setpoint = current_pose_;
    for (int i = 100; ros::ok() && i > 0; --i) {
        setpoint_pub_.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 切换到OFFBOARD模式并解锁
    last_request_ = ros::Time::now();
    while (ros::ok() && !current_state_.armed) {
        if (current_state_.mode != "OFFBOARD" && 
            (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
            if (setOffboardMode()) {
                last_request_ = ros::Time::now();
            }
        } else if (!current_state_.armed && 
                   (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
            if (armVehicle()) {
                last_request_ = ros::Time::now();
            }
        }
        setpoint_pub_.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("Starting mission...");
    flight_state_ = TAKEOFF;
    
    // 主控制循环
    while (ros::ok()) {
        ros::spinOnce();
        
        switch (flight_state_) {
            case TAKEOFF: {
                // 竖直起飞到目标高度（基于home位置）
                geometry_msgs::PoseStamped takeoff_target;
                takeoff_target.header.stamp = ros::Time::now();
                takeoff_target.header.frame_id = "world";
                takeoff_target.pose.position.x = home_pose_.pose.position.x;
                takeoff_target.pose.position.y = home_pose_.pose.position.y;
                takeoff_target.pose.position.z = home_pose_.pose.position.z + takeoff_height_;
                takeoff_target.pose.orientation = home_pose_.pose.orientation;
                
                setpoint_pub_.publish(takeoff_target);
                
                // 检查相对高度而不是绝对高度
                double relative_height = current_pose_.pose.position.z - home_pose_.pose.position.z;
                double height_error = fabs(relative_height - takeoff_height_);
                
                if (height_error < waypoint_tolerance_) {
                    ROS_INFO("Takeoff complete. Relative height: %.2f m (absolute: %.2f m)", 
                             relative_height, current_pose_.pose.position.z);
                    
                    // 生成圆圈航点（顺时针方向）
                    generateCircleWaypoints();
                    
                    // 找到离当前位置最近的航点
                    size_t nearest_idx = findNearestWaypoint();
                    
                    // 重新排序航点，从最近的点开始
                    reorderWaypointsFromIndex(nearest_idx);
                    
                    current_waypoint_idx_ = 0;
                    flight_state_ = CIRCLE_FLIGHT;
                    ROS_INFO("Starting circle flight from nearest waypoint (clockwise)...");
                } else {
                    // 每5秒打印一次进度
                    static ros::Time last_takeoff_print = ros::Time::now();
                    if ((ros::Time::now() - last_takeoff_print).toSec() > 5.0) {
                        ROS_INFO("Taking off... current: %.2f m, target: %.2f m (relative to home)", 
                                 relative_height, takeoff_height_);
                        last_takeoff_print = ros::Time::now();
                    }
                }
                break;
            }
            
            case CIRCLE_FLIGHT: {
                if (current_waypoint_idx_ >= circle_waypoints_.size()) {
                    ROS_INFO("Circle flight complete!");
                    flight_state_ = RETURN_HOME;
                    break;
                }
                
                geometry_msgs::PoseStamped target = circle_waypoints_[current_waypoint_idx_];
                target.header.stamp = ros::Time::now();
                setpoint_pub_.publish(target);
                
                double distance = calculateDistance(current_pose_.pose.position, 
                                                   target.pose.position);
                
                if (distance < waypoint_tolerance_) {
                    current_waypoint_idx_++;
                    if (current_waypoint_idx_ % 9 == 0) {  // 每1/4圈打印一次
                        ROS_INFO("Circle progress: %lu/%lu waypoints", 
                                current_waypoint_idx_, circle_waypoints_.size());
                    }
                }
                break;
            }
            
            case RETURN_HOME: {
                // 返回起飞点上方
                geometry_msgs::PoseStamped home_target;
                home_target.header.stamp = ros::Time::now();
                home_target.header.frame_id = "world";
                home_target.pose.position.x = home_pose_.pose.position.x;
                home_target.pose.position.y = home_pose_.pose.position.y;
                home_target.pose.position.z = home_pose_.pose.position.z + takeoff_height_;
                home_target.pose.orientation = home_pose_.pose.orientation;
                
                setpoint_pub_.publish(home_target);
                
                double distance = calculateDistance(current_pose_.pose.position,
                                                   home_target.pose.position);
                if (distance < waypoint_tolerance_) {
                    ROS_INFO("Returned to home position");
                    flight_state_ = LANDING;
                }
                break;
            }
            
            case LANDING: {
                // 降落到地面
                geometry_msgs::PoseStamped land_target;
                land_target.header.stamp = ros::Time::now();
                land_target.header.frame_id = "world";
                land_target.pose.position.x = home_pose_.pose.position.x;
                land_target.pose.position.y = home_pose_.pose.position.y;
                land_target.pose.position.z = home_pose_.pose.position.z + 0.2;  // 留0.2m余量
                land_target.pose.orientation = home_pose_.pose.orientation;
                
                setpoint_pub_.publish(land_target);
                
                double height = current_pose_.pose.position.z - home_pose_.pose.position.z;
                if (height < 0.5) {
                    ROS_INFO("Landing complete. Mission finished!");
                    flight_state_ = MISSION_COMPLETE;
                }
                break;
            }
            
            case MISSION_COMPLETE: {
                ROS_INFO_THROTTLE(5, "Mission complete. Node will continue running...");
                // 继续发送最后的设定点保持位置
                geometry_msgs::PoseStamped hold_target = home_pose_;
                hold_target.header.stamp = ros::Time::now();
                setpoint_pub_.publish(hold_target);
                break;
            }
            
            default:
                break;
        }
        
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "circle_flight_test");
    
    CircleFlightTest test;
    test.run();
    
    return 0;
}