#include <lib_library.h>
#include <ego_planner.h>
int mission_num = 0;  //确定当前任务

float target1_x = 0, target1_y = 0;
float target2_x = 0, target2_y = 0;
float target3_x = 0,target3_y = 0;
float target4_x = 0,target4_y = 0;


float err_max = 0.1;
int predict_where = 0;

void print_param()
{
    std::cout << "target1 : ( " << target1_x << ", " << target1_y << " )" << std::endl;
    std::cout << "target2 : ( " << target2_x << ", " << target2_y << " )" << std::endl;
    std::cout << "target3 : ( " << target3_x << ", " << target3_y << " )" << std::endl;
    std::cout << "target4 : ( " << target4_x << ", " << target4_y << " )" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
}
void print_publish()
{
    std::cout<< "setpoint_raw.x:"<<setpoint_raw.position.x<<std::endl;
    std::cout<< "setpoint_raw.y:"<<setpoint_raw.position.y<<std::endl;
    std::cout<< "setpoint_raw.z:"<<setpoint_raw.position.z<<std::endl;
    std::cout<< "setpoint_raw.yaw:"<<setpoint_raw.yaw<<std::endl;
}

int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "ego_planner");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅ego_planner规划出来的结果
    ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);

    // 发布ego_planner目标
    planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);

    ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

    finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);

    // 创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    // 创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);


    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);


    // 创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    // 创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);


    //  参数读取
    nh.param<float>("target1_x", target1_x, 0);
    nh.param<float>("target1_y", target1_y, 0);

    nh.param<float>("target2_x", target2_x, 0);
    nh.param<float>("target2_y", target2_y, 0);

    nh.param<float>("target3_x", target3_x, 0);
    nh.param<float>("target3_y", target3_y, 0);

    nh.param<float>("target4_x", target4_x, 0);
    nh.param<float>("target4_y", target4_y, 0);

    nh.param<float>("err_max", err_max, 0);

    print_param();

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
    if (choice != 1) return 0;

    // 等待连接到飞控   
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置:采用位置速度控制，世界坐标系
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    // 忽略速度/加速度等，仅设定位置与yaw

    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // 提前发布任务，防止控制真空期
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();

    //控制offboard和arm循环，初始化飞机状态
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.x = 0;
        setpoint_raw.position.y = 0;
        setpoint_raw.position.z = 1.2;
        setpoint_raw.yaw = 0;

        if(mission_pos_cruise(0, 0, 1.2, 0, 0.2)) {//不开避障，世界坐标系控制移动
            mission_num = 1; //开始任务
            break;
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // ==== 正式进入任务流程 ====
    // 定义任务计时标志位
    bool tmp_time_record_start_flag = false;
    // 定义任务计时变量
    ros::Time tmp_mission_success_time_record;

    int tmp_mission = 0; //显示切换任务变量，如果有切换变量时会打印当前任务切换信号

    last_request = ros::Time::now(); //更新记录时间，正式进入任务循环

    //任务开始
    while (ros::ok())
    {
        if (tmp_mission != mission_num)
        {
            tmp_mission = mission_num;
            printf("mission_num = %d\r\n", mission_num);
        }
        switch (mission_num)
        {
            //任务一：穿越两个障碍物到达识别区
            case 1: //悬停
                if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
                {
                    if (lib_time_record_func(1.5, ros::Time::now())){
                        mission_num = 10;
                        last_request = ros::Time::now();
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(3.0))
                {
                    mission_num = 10;
                    last_request = ros::Time::now();
                    ROS_WARN("Timed out!!!!");
                }
                break;

            case 10://ego_planner导航point1
                if (pub_ego_goal(target1_x, target1_y, ALTITUDE, err_max, 0)){
                    if (lib_time_record_func(0.5, ros::Time::now())){
                        mission_num = 11;
                        last_request = ros::Time::now();
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(5.0)){
                    mission_num = 11;
                    last_request = ros::Time::now();
                    ROS_WARN("Timed out!!!!");
                }
                break;

            case 11://ego_planner导航point2
                if (pub_ego_goal(target2_x, target2_y, ALTITUDE, err_max, 0)){
                    if (lib_time_record_func(0.5, ros::Time::now()))
                    {
                        mission_num = 12;
                        last_request = ros::Time::now();
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(5.0)){
                    mission_num = 12;
                    last_request = ros::Time::now();
                    ROS_WARN("Timed out!!!!");
                }
                break;

            case 12://ego_planner导航point3
                if (pub_ego_goal(target3_x, target3_y, ALTITUDE, err_max, 0)){
                    if (lib_time_record_func(0.5, ros::Time::now())){
                        mission_num = 13;
                        last_request = ros::Time::now();
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(5.0)){
                    mission_num = 13;
                    last_request = ros::Time::now();
                    ROS_WARN("Timed out!!!!");
                }
                break;

            case 13://ego_planner导航point4
                if (pub_ego_goal(target4_x, target4_y, ALTITUDE, err_max, 0)){
                    if (lib_time_record_func(0.5, ros::Time::now())){
                        mission_num = 14;
                        last_request = ros::Time::now();
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(5.0)){
                    mission_num = 14;
                    last_request = ros::Time::now();
                    ROS_WARN("Timed out!!!!");
                }
                break;

            case 100:
                if(mission_pos_cruise(0, 0, 0.05, 3.14, err_max))
                {
                    if (lib_time_record_func(1.0, ros::Time::now())){
                        ROS_INFO("AUTO.LAND");
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        set_mode_client.call(offb_set_mode);
                        mission_num = -1;
                    }
                }
                else if(ros::Time::now() - last_request >= ros::Duration(3.0))
                {
                    ROS_INFO("AUTO.LAND");
                    offb_set_mode.request.custom_mode = "AUTO.LAND";
                    set_mode_client.call(offb_set_mode);
                    mission_num = -1;
                }
                break;
        }
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        print_publish();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}