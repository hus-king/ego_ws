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
#include <string>
#include <geometry_msgs/Twist.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
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
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
	current_state = *msg;
	//	std::cout << "ok " << std::endl;
}

/************************************************************************
函数功能2：回调函数接收无人机的里程计信息
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能3：自主巡航，发布目标位置，控制无人机到达目标，采用local坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能4：自主巡航，发布目标位置，控制无人机到达目标，采用机体坐标系位置控制
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能5: 获取yolo识别目标的位置信息
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能6: 目标识别，采用速度控制运动，任务是识别到目标后，保持无人机正对着目标
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能7: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能8: //无人机追踪，仅水平即Y方向，通常用于无人机穿越圆框、方框等使用
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能9: 用于接收二维码识别返回值
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
ar_track_alvar_msgs::AlvarMarker marker;
float marker_x = 0, marker_y = 0, marker_z = 0;
bool marker_found = false;
int ar_track_id_current = 0;
void ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
void ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	int count = msg->markers.size();
	if (count != 0)
	{
		marker_found = true;
		for (int i = 0; i < count; i++)
		{
			marker = msg->markers[i];
			ar_track_id_current = marker.id;
		}
	}
	else
	{
		marker_found = false;
	}
}

/************************************************************************
函数功能10: 二维码降落,此处代码尚未实际验证，待验证
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
float ar_lable_land_last_position_x = 0;
float ar_lable_land_last_position_y = 0;
bool ar_lable_land_init_position_flag = false;
float track_speed = 0;
bool ar_lable_land(float marker_error_max, int ar_track_id, float altitude, int flag = 0);
bool ar_lable_land(float marker_error_max, int ar_track_id, float altitude, int flag)
{
	if (ar_lable_land_init_position_flag == false)
	{
		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
		ar_lable_land_last_position_y = local_pos.pose.pose.position.y;
		ar_lable_land_init_position_flag = true;
		ROS_INFO("进入二维码识别和降落");
	}
	// 识别到指定的id才进入这个循环，否则不控制
	if (marker.id == ar_track_id && (marker.id != 0))
	{
		// 此处根本摄像头安装方向，进行静态坐标转换
		marker_x = marker.pose.pose.position.x;
		marker_y = marker.pose.pose.position.y;
		marker_z = marker.pose.pose.position.z;

		printf("ar_track_id = %d\r\n", ar_track_id);
		printf("marker_x = %f\r\n", marker_x);
		printf("marker_y = %f\r\n", marker_y);
		printf("marker_z = %f\r\n", marker_z);
		printf("track_speed = %f\r\n", track_speed);

		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
		ar_lable_land_last_position_y = local_pos.pose.pose.position.y;

		float tmp_marker_x = marker_x;
		float tmp_marker_y = marker_y;
		if (flag == 2)
		{
			tmp_marker_y += -0.07;
		}

		if (fabs(tmp_marker_x) < marker_error_max && fabs(tmp_marker_y) < marker_error_max)
		{
			ar_lable_land_init_position_flag = false;
			ROS_INFO("到达目标的正上/前方");
			return true;
		}
		else
		{
			if (flag != 2)
			{
				// 摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
				// 无人机左右移动速度控制
				if (marker_x >= marker_error_max)
				{
					// setpoint_raw.position.y = local_pos.pose.pose.position.y - 0.1;
					setpoint_raw.velocity.y = -track_speed;
				}
				else if (marker_x <= -marker_error_max)
				{
					// setpoint_raw.position.y = local_pos.pose.pose.position.y + 0.1;
					setpoint_raw.velocity.y = track_speed;
				}
				else
				{
					// setpoint_raw.position.y = ar_lable_land_last_position_y;
					setpoint_raw.velocity.y = 0;
				}
				// 无人机前后移动速度控制
				if (marker_y >= marker_error_max)
				{
					// setpoint_raw.position.x = local_pos.pose.pose.position.x - 0.1;
					setpoint_raw.velocity.x = -track_speed;
				}
				else if (marker_y <= -marker_error_max)
				{
					// setpoint_raw.position.x = local_pos.pose.pose.position.x + 0.1;
					setpoint_raw.velocity.x = track_speed;
				}
				else
				{
					// setpoint_raw.position.x = ar_lable_land_last_position_x;
					setpoint_raw.velocity.x = 0;
				}
				if (flag)
				{
					std::cout << "reverse" << std::endl;
					setpoint_raw.velocity.x = -setpoint_raw.velocity.x;
					setpoint_raw.velocity.y = -setpoint_raw.velocity.y;
				}
			}
			else
			{
				// 摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
				// 无人机左右移动速度控制
				if (marker_x >= marker_error_max)
				{
					// setpoint_raw.position.y = local_pos.pose.pose.position.y - 0.1;
					setpoint_raw.velocity.y = -track_speed;
				}
				else if (marker_x <= -marker_error_max)
				{
					// setpoint_raw.position.y = local_pos.pose.pose.position.y + 0.1;
					setpoint_raw.velocity.y = track_speed;
				}
				else
				{
					// setpoint_raw.position.y = ar_lable_land_last_position_y;
					setpoint_raw.velocity.y = 0;
				}
				// 无人机前后移动速度控制
				if (marker_y >= marker_error_max + 0.07)
				{
					// setpoint_raw.position.x = local_pos.pose.pose.position.x - 0.1;
					setpoint_raw.velocity.x = -track_speed;
				}
				else if (marker_y <= -marker_error_max + 0.07)
				{
					// setpoint_raw.position.x = local_pos.pose.pose.position.x + 0.1;
					setpoint_raw.velocity.x = track_speed;
				}
				else
				{
					// setpoint_raw.position.x = ar_lable_land_last_position_x;
					setpoint_raw.velocity.x = 0;
				}
			}
		}
		setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 +*/ 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	}
	else
	{
		setpoint_raw.position.x = ar_lable_land_last_position_x;
		setpoint_raw.position.y = ar_lable_land_last_position_y;
		setpoint_raw.type_mask = /*1 + 2 + 4*/ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
	}
	// setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 /*+ 1024 */+ 2048;
	setpoint_raw.position.z = altitude;
	setpoint_raw.coordinate_frame = 1;

	double current_yaw, a, b;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(a, b, current_yaw);

	setpoint_raw.yaw = current_yaw;
	return false;
}

/************************************************************************
函数功能11: move_base速度转换
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能12: move_base
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能13: 穿越圆框，发布圆框后方目标点，效果良好，有时间以后可以继续优化
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能14:降落
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能:ego_planner导航
//1、定义变量
//2、函数声明
//3、函数定义
*************************************************************************/
quadrotor_msgs::PositionCommand ego_sub;
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
void ego_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
	ego_sub = *msg;
}

/************************************************************************
函数功能: ego_planner是否规划出航线
//1、定义变量
//2、函数声明
//3、函数定义
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
函数功能:ego_planner发布目标点函数
//1、定义变量
//2、函数声明
//3、函数定义
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

/*-----------------------------视觉识别--------------------------*/
typedef struct TARGET{
    string text;
    float x[4];
    float y[4];
} TARGET;
float ocr_threshold = 0.50;// OCR确认识别的阈值
vector<paddle_ocr_ros::OCRItem> test_v;
bool front_ocr_flag = false;
bool down_ocr_flag = false;

bool front_found = false;
bool down_found = false;

string target_text = "A";
TARGET down_target;
enum CamType{
    CAM_DOWN,
    CAM_FRONT
};

string available_targets[] = {"A", "B", "C", "c"};
int front_where = -1; //0:left 1:middle 2:right

bool quadrilateral_validation(const boost::array<geometry_msgs::Point, 4>& polygon){
    return polygon.size() == 4;
}

float calculateQuadrilateralArea(const boost::array<geometry_msgs::Point, 4>& polygon) 
{
    // 有效性检查
    if (!quadrilateral_validation(polygon)) {
        ROS_ERROR("多边形坐标数量错误，需要4个点实际收到 %zu 个", polygon.size());
        return 0.0f;
    }
    
    // 使用鞋带公式计算面积
    float area = 0.0;
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        area += (polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y);
    }
    return std::abs(area) * 0.5f;
}


//判断是否开启
bool ocr_result_validation(const paddle_ocr_ros::OCRResult::ConstPtr &msg, CamType camtype){
    if(!(camtype == CAM_DOWN? down_ocr_flag: front_ocr_flag)){
        return false;
    }
    if(msg->items.empty()){
        return false;
    }
    return true;
}


bool l2r_cmp(const paddle_ocr_ros::OCRItem &box1,const paddle_ocr_ros::OCRItem &box2){
    float mid1=0.0f,mid2=0.0f;
    for(int i=0; i<4; ++i){
        mid1+=box1.polygon[i].x;
        mid2+=box2.polygon[i].x;
    }
    return mid1<mid2;
}


bool ocr_item_validation(const paddle_ocr_ros::OCRItem &msg, CamType camtype){
    if(msg.confidence <= ocr_threshold){
        return false;
    }
    if(msg.polygon.size()!= 4){
        return false;
    }

    bool targ_check = false;
    for(string target:available_targets){
        if(target == msg.text){
            targ_check = true;
        }
    }
    if(!targ_check){
        return false;//添加干扰排除
    }
    switch(camtype){
        case CAM_DOWN:{
            if(!down_found)
                down_found = true;
            break;
        }
        case CAM_FRONT:{
            break;
        }
    }
    return true;
}



void target_constructor(const paddle_ocr_ros::OCRItem &msg, CamType camtype){
    if(camtype == CAM_DOWN){
        down_target.text = msg.text;
        if(down_target.text=="c"){
            down_target.text=string("C");
        }
        for (int i = 0; i < 4; ++i) {
            down_target.x[i] = msg.polygon[i].x;
            down_target.y[i] = msg.polygon[i].y;
        }
    }
}



//向下读取回调
void down_ocr_cb(const paddle_ocr_ros::OCRResult::ConstPtr &msg){
    CamType camtype = CAM_DOWN;
    if(!ocr_result_validation(msg, camtype)) return;

    down_found = false;
    float biggest = 0.0f;
    int index = -1,items_size = msg->items.size();

    for(int i=0;i<items_size;++i){
        if(!ocr_item_validation(msg->items[i], camtype)){
            continue;
        }
		int area = calculateQuadrilateralArea((msg->items[i]).polygon);
        if(area > biggest){
            biggest = area;
            index = i;
        }
    }
    if(index<0 || index >= items_size){
        down_found = false;
        return;
    }
    target_constructor(msg->items[index], camtype);
}

//向前读取回调
void front_ocr_cb(const paddle_ocr_ros::OCRResult::ConstPtr &msg){

    CamType camtype = CAM_FRONT;
    if(!ocr_result_validation(msg, camtype)) return;

    front_found = false;
    float biggest = 0.0f;
    int items_size = msg->items.size();
    vector<paddle_ocr_ros::OCRItem> v;

    for(int i=0;i<items_size;++i){
        if(!ocr_item_validation(msg->items[i], camtype)){
            continue;
        }
        v.push_back(msg->items[i]);
    }
    sort(v.begin(),v.end(),l2r_cmp);
    if(v.size()!=3){
        ROS_ERROR("EXACT MATCH FAILED: filtered target num is not 3!");
		return;
    }
	else{
		for(int i=0;i<v.size();++i){
			ROS_WARN("Vector Element %d: %s",i, v[i].text.c_str());
		}
		test_v = v;
	}
    for(int i=0;i<v.size();++i){
        if(v[i].text == target_text){
            front_where = i;
			front_found = true;
			break;
        }
        if(v[i].text=="c" && target_text == "C"){
            front_where = i;
			front_found = true;
			break;
        }
    }

}
