#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <geometry_msgs/Twist.h>
#include "quadrotor_msgs/PositionCommand.h"

using namespace std;

#define ALTITUDE 0.55

mavros_msgs::PositionTarget setpoint_raw;
ros::Publisher planner_goal_pub;
ros::Publisher finish_ego_pub;
std_msgs::Bool finish_ego_flag;

float map_size_z = 0.65;
float ground_height = 0.3;

/************************************************************************
函数功能1：无人机状态回调函数
功能描述：接收并处理MAVROS状态信息，更新无人机连接状态、飞行模式等
输入参数：
  - msg: mavros_msgs::State::ConstPtr 类型，包含无人机状态信息
返回值：无
使用说明：作为ROS订阅者回调函数，自动接收/mavros/state话题数据
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
    //	std::cout << "ok " << std::endl;
}

/************************************************************************
函数功能2：无人机里程计信息回调函数
功能描述：接收并处理无人机位置、姿态信息，进行四元数到欧拉角转换，记录起飞初始位置
输入参数：
  - msg: nav_msgs::Odometry::ConstPtr 类型，包含位置、速度、姿态信息
返回值：无
使用说明：作为ROS订阅者回调函数，自动接收/mavros/local_position/odom话题数据
          首次接收到有效高度数据时会记录起飞初始位置作为参考点
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}


/************************************************************************
函数功能3：自主巡航位置控制函数
功能描述：控制无人机飞行到指定目标点，采用local坐标系位置控制模式
输入参数：
  - x: float 类型，目标点X坐标(相对于起飞点)
  - y: float 类型，目标点Y坐标(相对于起飞点)  
  - z: float 类型，目标点Z坐标(相对于起飞点)
  - yaw: float 类型，目标偏航角(弧度)
  - error_max: float 类型，位置误差阈值，小于此值认为到达目标
返回值：bool 类型，true表示已到达目标点，false表示仍在飞行中
使用说明：函数会自动记录初始位置，目标坐标会加上起飞点偏移量转换为绝对坐标
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = yaw;

    ROS_INFO("Flying to ( %.2f, %.2f )", x, y);

    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数功能4：机体坐标系相对位置控制函数
功能描述：控制无人机相对当前位置移动指定距离，采用机体坐标系位置控制
输入参数：
  - x: float 类型，相对于当前位置的X轴偏移量
  - y: float 类型，相对于当前位置的Y轴偏移量
  - z: float 类型，目标绝对高度
  - yaw: float 类型，目标偏航角(弧度)
  - error_max: float 类型，位置误差阈值，小于此值认为到达目标
返回值：bool 类型，true表示已到达目标点，false表示仍在飞行中
使用说明：函数在首次调用时记录起始位置，后续以此为基准进行相对移动控制
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
float current_position_cruise_last_position_yaw = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float yaw, float error_max)
{
    if (current_position_cruise_flag == false)
    {
        current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
        current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
        current_position_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = current_position_cruise_last_position_x + x;
    setpoint_raw.position.y = current_position_cruise_last_position_y + y;
    setpoint_raw.position.z = z;
    setpoint_raw.yaw = yaw;
    double current_yaw, a, b;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

    std::cout << "x: " << local_pos.pose.pose.position.x << std::endl;

    if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(current_yaw - yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        current_position_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数功能5: YOLO目标识别位置信息回调函数
功能描述：接收YOLO检测到的目标物体在图像中的位置信息
输入参数：
  - msg: geometry_msgs::PointStamped::ConstPtr 类型，包含目标物体位置信息
返回值：无
使用说明：作为ROS订阅者回调函数，用于获取视觉检测结果的位置数据
*************************************************************************/
geometry_msgs::PointStamped object_pos;
double position_detec_x = 0;
double position_detec_y = 0;
double position_detec_z = 0;
void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr &msg);
void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    object_pos = *msg;
}

/************************************************************************
函数功能6: 目标识别与速度跟踪控制函数
功能描述：基于视觉识别结果，采用速度控制使无人机保持在目标正上方/前方
输入参数：
  - str: string 类型，目标识别字符串，用于匹配特定目标
  - yaw: float 类型，期望偏航角(弧度)
  - altitude: float 类型，飞行高度(相对于起飞点)
  - speed: float 类型，调整速度大小
  - error_max: float 类型，图像中心偏差阈值，小于此值认为对准目标
返回值：bool 类型，true表示已对准目标，false表示仍在调整中
使用说明：假设摄像头向下安装，图像中心为(320,240)，函数会自动调整XY速度使目标居中
*************************************************************************/
float object_recognize_track_vel_last_time_position_x = 0;
float object_recognize_track_vel_last_time_position_y = 0;
bool object_recognize_track_vel_flag = false;
bool object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max);
bool object_recognize_track_vel(string str, float yaw, float altitude, float speed, float error_max)
{
    // 此处false主要是为了获取任务初始时候的位置信息
    if (object_recognize_track_vel_flag == false)
    {
        // 获取初始位置，防止无人机飘
        object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
        object_recognize_track_vel_flag = true;
        ROS_INFO("开始识别目标并且保持跟踪");
    }
    // 此处首先判断是否识别到指定的目标
    if (object_pos.header.frame_id == str)
    {
        // 此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
        // 获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z;
        if (fabs(position_detec_x - 320) < error_max && fabs(position_detec_y - 240) < error_max)
        {
            ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
            return true;
        }
        else
        {
            // 摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
            if (position_detec_x - 320 >= error_max)
            {
                setpoint_raw.velocity.y = -speed;
            }
            else if (position_detec_x - 320 <= -error_max)
            {
                setpoint_raw.velocity.y = speed;
            }
            else
            {
                setpoint_raw.velocity.y = 0;
            }
            // 无人机前后移动速度控制
            if (position_detec_y - 240 >= error_max)
            {
                setpoint_raw.velocity.x = -speed;
            }
            else if (position_detec_y - 240 <= -error_max)
            {
                setpoint_raw.velocity.x = speed;
            }
            else
            {
                setpoint_raw.velocity.x = 0;
            }
            // 机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
            setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 + 1024 + 2048;
            setpoint_raw.coordinate_frame = 8;
            setpoint_raw.position.z = init_position_z_take_off + altitude;
        }
    }
    else
    {
        setpoint_raw.position.x = object_recognize_track_vel_last_time_position_x;
        setpoint_raw.position.y = object_recognize_track_vel_last_time_position_y;
        setpoint_raw.type_mask = /*1 + 2 + 4*/ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024*/ + 2048;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.z = init_position_z_take_off + altitude;
        setpoint_raw.yaw = yaw;
    }
    return false;
}

/************************************************************************
函数功能7: 单方向目标跟踪控制函数
功能描述：基于视觉识别结果，仅在Y轴(水平)方向进行位置跟踪，常用于穿越圆框、方框等场景
输入参数：
  - str: string 类型，目标识别字符串，用于匹配特定目标
  - yaw: float 类型，期望偏航角(弧度)
  - reverse: int 类型，反向控制标志，1为正向，-1为反向
  - altitude: float 类型，飞行高度
  - error_max: float 类型，图像中心偏差阈值，小于此值认为对准目标
  - ctrl_coef: float 类型，控制系数，影响调整幅度
返回值：bool 类型，true表示已对准目标，false表示仍在调整中
使用说明：仅控制Y轴方向，X轴保持初始位置，适用于需要精确水平对齐的场景
*************************************************************************/
float object_recognize_track_last_time_position_x = 0;
float object_recognize_track_last_time_position_y = 0;
bool object_recognize_track_flag = false;
bool object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef);
bool object_recognize_track(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef)
{
    if (object_recognize_track_flag == false)
    {
        object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
        object_recognize_track_flag = true;
        ROS_INFO("开始识别目标并且保持跟踪");
    }
    // 此处首先判断是否识别到指定的目标
    if (object_pos.header.frame_id == str)
    {
        // 此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
        // 获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z;

        // 摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
        if (position_detec_x - 320 >= error_max)
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef * reverse;
        }
        else if (position_detec_x - 320 <= -error_max)
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef * reverse;
        }
        else
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y;
        }

        if (fabs(position_detec_x - 320) < error_max)
        {
            ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
            return true;
        }
    }
    else
    {
        setpoint_raw.position.x = object_recognize_track_last_time_position_x;
        setpoint_raw.position.y = object_recognize_track_last_time_position_y;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = object_recognize_track_last_time_position_x;
    setpoint_raw.position.z = 0 + altitude;
    setpoint_raw.yaw = yaw;
    return false;
}

/************************************************************************
函数功能8: 全方向目标跟踪控制函数
功能描述：基于视觉识别结果，在X、Y两个方向同时进行位置跟踪，实现全方位对准
输入参数：
  - str: string 类型，目标识别字符串，用于匹配特定目标
  - yaw: float 类型，期望偏航角(弧度)
  - reverse: int 类型，反向控制标志，1为正向，-1为反向
  - altitude: float 类型，飞行高度
  - error_max: float 类型，图像中心偏差阈值，小于此值认为对准目标
  - ctrl_coef: float 类型，控制系数，影响调整幅度
返回值：bool 类型，true表示已对准目标，false表示仍在调整中
使用说明：同时控制X、Y轴方向，实现精确的二维位置跟踪，适用于精准降落等场景
*************************************************************************/
float object_recognize_track_omni_last_time_position_x = 0;
float object_recognize_track_omni_last_time_position_y = 0;
bool object_recognize_track_omni_flag = false;
bool object_recognize_track_omni(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef);
bool object_recognize_track_omni(string str, float yaw, int reverse, float altitude, float error_max, float ctrl_coef)
{
    if (object_recognize_track_omni_flag == false)
    {
        object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
        object_recognize_track_omni_flag = true;
        ROS_INFO("开始识别目标并且保持跟踪");
    }
    // 此处首先判断是否识别到指定的目标
    if (object_pos.header.frame_id == str)
    {
        object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
        // 获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z;

        // 摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
        if (position_detec_x - 320 >= error_max)
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef * reverse;
        }
        else if (position_detec_x - 320 <= -error_max)
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef * reverse;
        }
        else
        {
            setpoint_raw.position.y = local_pos.pose.pose.position.y;
        }
        if (position_detec_y - 240 >= error_max)
        {
            setpoint_raw.position.x = local_pos.pose.pose.position.x - ctrl_coef * reverse;
        }
        else if (position_detec_y - 240 <= -error_max)
        {
            setpoint_raw.position.x = local_pos.pose.pose.position.x + ctrl_coef * reverse;
        }
        else
        {
            setpoint_raw.position.x = local_pos.pose.pose.position.x;
        }

        if (fabs(position_detec_x - 320) < error_max && fabs(position_detec_y - 240) < error_max)
        {
            ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
            return true;
        }
    }
    else
    {
        setpoint_raw.position.x = object_recognize_track_omni_last_time_position_x;
        setpoint_raw.position.y = object_recognize_track_omni_last_time_position_y;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.z = 0 + altitude;
    setpoint_raw.yaw = yaw;
    return false;
}

/************************************************************************
函数功能10: MoveBase导航避障控制函数
功能描述：使用ROS navigation stack进行路径规划和避障导航到目标点
输入参数：
  - x: float 类型，目标点X坐标(地图坐标系)
  - y: float 类型，目标点Y坐标(地图坐标系)
  - z: float 类型，目标点Z坐标(通常不使用)
  - yaw: float 类型，目标偏航角(弧度)
返回值：bool 类型，true表示已到达目标点，false表示仍在导航中
使用说明：首次调用时发布导航目标，后续检查是否到达，适用于复杂环境的避障导航
*************************************************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool mission_obs_avoid_flag = false;
bool mission_obs_avoid(float x, float y, float z, float yaw)
{

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    if (mission_obs_avoid_flag == false)
    {
        mission_obs_avoid_flag = true;
        ROS_INFO("发布避障导航目标点 x = %f, y = %f, z = %f, yaw = %f", x, y, z, yaw);
        move_base_msgs::MoveBaseGoal first_goal;
        first_goal.target_pose.header.frame_id = "map";
        first_goal.target_pose.header.stamp = ros::Time::now();
        first_goal.target_pose.pose.position.x = x;
        first_goal.target_pose.pose.position.y = y;
        first_goal.target_pose.pose.position.z = z;
        first_goal.target_pose.pose.orientation.w = 1;
        ac.sendGoal(first_goal);
    }

    if (fabs(local_pos.pose.pose.position.x - x) < 0.2 && fabs(local_pos.pose.pose.position.y - y) < 0.2)
    {
        ROS_INFO("到达目标点，自主导航任务完成");
        mission_obs_avoid_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数功能11: MoveBase速度指令转换函数
功能描述：将MoveBase发布的速度指令转换为PX4飞控可识别的控制指令
输入参数：
  - msg: geometry_msgs::Twist 类型，包含线速度和角速度指令
返回值：无
使用说明：作为ROS订阅者回调函数，当导航避障功能启用时自动转换速度指令
          转换后的指令通过setpoint_raw发布给飞控执行
*************************************************************************/
void cmd_to_px4(const geometry_msgs::Twist &msg);
void cmd_to_px4(const geometry_msgs::Twist &msg)
{
    if (mission_obs_avoid_flag)
    {
        setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 /*+ 2048*/;
        setpoint_raw.coordinate_frame = 8;
        setpoint_raw.velocity.x = msg.linear.x;
        setpoint_raw.velocity.y = msg.linear.y;
        setpoint_raw.position.z = ALTITUDE;
        //		setpoint_raw.yaw_rate = 0;
        setpoint_raw.yaw_rate = msg.angular.z;
        ROS_INFO("move_base working, vel_x:%.2f, vel_y:%.2f, yaw_rate:%.2f", msg.linear.x, msg.linear.y, msg.angular.z);
    }
}

/************************************************************************
函数功能12: 穿越障碍物控制函数
功能描述：控制无人机穿越圆框等障碍物，通过发布障碍物后方目标点实现穿越
输入参数：
  - pos_x: float 类型，相对于起始位置的X轴偏移量
  - pos_y: float 类型，相对于起始位置的Y轴偏移量
  - z: float 类型，目标绝对高度
  - yaw: float 类型，目标偏航角(弧度)
返回值：bool 类型，true表示已穿越到目标点，false表示仍在飞行中
使用说明：函数效果良好，记录初始位置后控制飞行到相对位置，适用于障碍物穿越场景
          后续可根据需要继续优化算法
*************************************************************************/
float target_through_init_position_x = 0;
float target_through_init_position_y = 0;
bool target_through_init_position_flag = false;
bool target_through(float pos_x, float pos_y, float z, float yaw);
bool target_through(float pos_x, float pos_y, float z, float yaw)
{
    // 初始化位置一次即可，用于获取无人机初始位置
    if (!target_through_init_position_flag)
    {
        target_through_init_position_x = local_pos.pose.pose.position.x;
        target_through_init_position_y = local_pos.pose.pose.position.y;
        target_through_init_position_flag = true;
    }
    setpoint_raw.position.x = target_through_init_position_x + pos_x;
    setpoint_raw.position.y = target_through_init_position_y + pos_y;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.z = z;
    setpoint_raw.yaw = yaw;

    double current_yaw, a, b;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(a, b, current_yaw);
    if (fabs(local_pos.pose.pose.position.x - pos_x - target_through_init_position_x) < 0.1 && fabs(local_pos.pose.pose.position.y - pos_y - target_through_init_position_y) < 0.1 && fabs(current_yaw - yaw) < 0.1)
    {
        target_through_init_position_flag = false;
        ROS_INFO("到达目标点，穿越圆框任务完成");
        return true;
    }
    return false;
}

/************************************************************************
函数功能13: 精准降落控制函数
功能描述：控制无人机在当前位置垂直下降至地面进行精准降落
输入参数：无
返回值：无
使用说明：函数会记录当前X、Y位置并保持不变，仅降低Z轴高度至-0.15米
          适用于需要在特定位置精确降落的场景
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
void precision_land();
void precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
}

/************************************************************************
函数功能14: EGO规划器位置指令回调函数
功能描述：接收EGO路径规划器发布的位置和速度指令
输入参数：
  - msg: quadrotor_msgs::PositionCommand::ConstPtr 类型，包含目标位置、速度、偏航角等信息
返回值：无
使用说明：作为ROS订阅者回调函数，接收/position_cmd话题的路径规划结果
*************************************************************************/
quadrotor_msgs::PositionCommand ego_sub;
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    ego_sub = *msg;
}

/************************************************************************
函数功能15: EGO规划器轨迹接收状态回调函数
功能描述：接收EGO规划器是否成功规划出可行轨迹的状态信息
输入参数：
  - msg: std_msgs::Bool::ConstPtr 类型，true表示已规划出轨迹，false表示未规划出轨迹
返回值：无
使用说明：作为ROS订阅者回调函数，接收/rec_traj话题的规划状态
          用于判断是否可以执行轨迹跟踪控制
*************************************************************************/
bool rec_traj_flag = false;
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg);
void rec_traj_cb(const std_msgs::Bool::ConstPtr &msg)
{
    rec_traj_flag = msg->data;
}

void PI_attitude_control()
{
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.type_mask = 0b101111100011; // vx vy z yaw
    setpoint_raw.velocity.x = 0.85 * ego_sub.velocity.x + (ego_sub.position.x - local_pos.pose.pose.position.x) * 1;
    setpoint_raw.velocity.y = 0.85 * ego_sub.velocity.y + (ego_sub.position.y - local_pos.pose.pose.position.y) * 1;
    setpoint_raw.position.z = ego_sub.position.z;
    setpoint_raw.yaw = ego_sub.yaw;

    ROS_INFO("ego: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", ego_sub.velocity.x, ego_sub.velocity.y, ego_sub.position.z, ego_sub.yaw);
    ROS_INFO("ego: x = %.2f, y = %.2f, z = %.2f, yaw = %.2f", ego_sub.position.x, ego_sub.position.y, ego_sub.position.z, ego_sub.yaw);
    ROS_INFO("已触发控制器: vel_x = %.2f, vel_y = %.2f, z = %.2f, yaw = %.2f", setpoint_raw.velocity.x, setpoint_raw.velocity.y, setpoint_raw.position.z, setpoint_raw.yaw);
}

/************************************************************************
函数功能16: EGO规划器目标点发布与跟踪控制函数
功能描述：发布EGO规划器目标点并执行轨迹跟踪控制，实现智能路径规划和飞行
输入参数：
  - x: float 类型，目标点X坐标(世界坐标系)
  - y: float 类型，目标点Y坐标(世界坐标系)
  - z: float 类型，目标点Z坐标(世界坐标系)
  - err_max: float 类型，位置误差阈值，小于此值认为到达目标
  - first_target: int 类型，特殊目标标志(默认0，可选1或2用于特定偏航角控制)
返回值：bool 类型，true表示已到达目标点，false表示仍在飞行中
使用说明：结合EGO规划器进行避障路径规划，包含PI控制器进行轨迹跟踪
          支持高度限制和地面高度保护，确保飞行安全
*************************************************************************/
float before_ego_pose_x = 0;
float before_ego_pose_y = 0;
float before_ego_pose_z = 0;
bool pub_ego_goal_flag = false;
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target = 0);
bool pub_ego_goal(float x, float y, float z, float err_max, int first_target)
{
    before_ego_pose_x = local_pos.pose.pose.position.x;
    before_ego_pose_y = local_pos.pose.pose.position.y;
    before_ego_pose_z = local_pos.pose.pose.position.z;

    double current_yaw, a, b;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

    geometry_msgs::PoseStamped target_point;
    if (pub_ego_goal_flag == false)
    {
        pub_ego_goal_flag = true;
        target_point.pose.position.x = x;
        target_point.pose.position.y = y;
        target_point.pose.position.z = z;

        planner_goal_pub.publish(target_point);
        ROS_INFO("发送目标点( %f, %f, %f )", x, y, z);

        finish_ego_flag.data = false;
        finish_ego_pub.publish(finish_ego_flag);
    }

    if (rec_traj_flag == true)
    {
        PI_attitude_control();

        if (ego_sub.position.z > map_size_z)
        {
            setpoint_raw.position.z = map_size_z;
            std::cout << "exceed map_size_z" << std::endl;
        }
        else if (ego_sub.position.z < ground_height)
        {
            setpoint_raw.position.z = ground_height;
            std::cout << "lower than ground height" << std::endl;
        }

        if (first_target == 1)
        {
            std::cout << "get traj" << std::endl;
            std::cout << "ego_x : " << ego_sub.position.x << std::endl;
            std::cout << "ego_y : " << ego_sub.position.y << std::endl;
            std::cout << "ego_z : " << ego_sub.position.z << std::endl;
            std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
            setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
            setpoint_raw.coordinate_frame = 1;
            setpoint_raw.position.x = ego_sub.position.x;
            setpoint_raw.position.y = ego_sub.position.y;
            setpoint_raw.position.z = ego_sub.position.z;
            setpoint_raw.velocity.x = ego_sub.velocity.x;
            setpoint_raw.velocity.y = ego_sub.velocity.y;
            setpoint_raw.velocity.z = ego_sub.velocity.z;
            setpoint_raw.yaw = 0;
        }
        if (first_target == 2)
        {
            std::cout << "get traj" << std::endl;
            std::cout << "ego_x : " << ego_sub.position.x << std::endl;
            std::cout << "ego_y : " << ego_sub.position.y << std::endl;
            std::cout << "ego_z : " << ego_sub.position.z << std::endl;
            std::cout << "ego_yaw : " << ego_sub.yaw << std::endl;
            setpoint_raw.type_mask = 0b101111000000; // 101 111 000 000  vx vy　vz x y z+ yaw
            setpoint_raw.coordinate_frame = 1;
            setpoint_raw.position.x = ego_sub.position.x;
            setpoint_raw.position.y = ego_sub.position.y;
            setpoint_raw.position.z = ego_sub.position.z;
            setpoint_raw.velocity.x = ego_sub.velocity.x;
            setpoint_raw.velocity.y = ego_sub.velocity.y;
            setpoint_raw.velocity.z = ego_sub.velocity.z;
            setpoint_raw.yaw = 3.14;
        }
    }
    else
    {
        std::cout << "not rec_tarj" << std::endl;
        setpoint_raw.position.x = before_ego_pose_x;
        setpoint_raw.position.y = before_ego_pose_y;
        setpoint_raw.position.z = before_ego_pose_z;
        setpoint_raw.yaw = current_yaw;
        setpoint_raw.type_mask = 0b101111111000; // 101 111 111 000  x y z+ yaw
        setpoint_raw.coordinate_frame = 1;
    }

    setpoint_raw.header.stamp = ros::Time::now();

    if (fabs(local_pos.pose.pose.position.x - x) < err_max && fabs(local_pos.pose.pose.position.y - y) < err_max)
    {
        ROS_INFO("到达目标点, ego_planner导航任务完成");
        pub_ego_goal_flag = false;

        finish_ego_flag.data = true;
        finish_ego_pub.publish(finish_ego_flag);

        return true;
    }
    return false;
}