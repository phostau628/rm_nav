// 包含自定义的头文件，声明了Navi_cal类
#include "navi_cal/navi_cal.hpp"

// 定义命名空间taurus，避免命名冲突
namespace taurus
{
    // Navi_cal类的构造函数，初始化ROS 2节点
    // 参数：NodeOptions - ROS 2节点的配置选项
    Navi_cal::Navi_cal(const rclcpp::NodeOptions &options) : Node("Navi_cal",options)
    {   
        // 调试日志（已注释）
        // RCLCPP_INFO(this->get_logger(),"Navi_cal");
        
        // 设置定时器周期为40毫秒（25Hz）
        auto period = std::chrono::milliseconds(40);
        // 创建定时器，定时触发timerCallback回调函数
        timer_ = this->create_wall_timer(period,std::bind(&Navi_cal::timerCallback,this));
        
        // 创建TF2缓冲区，用于存储和查询坐标变换，关联节点的时钟
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        // 创建TF2监听器，监听并接收TF变换数据到缓冲区
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建发布者，发布/fdb话题，消息类型为auto_aim_interfaces::msg::Fdb，队列大小10
        fdb_pub_ = this->create_publisher<auto_aim_interfaces::msg::Fdb>("/fdb", 10);
        
        // 创建订阅者，订阅gimbal_fdb话题（云台反馈）
        // QoS设置为SensorDataQoS（适用于传感器数据），回调函数为GimbalFdbCallback
        gimbal_fdb_sub_ = this->create_subscription<auto_aim_interfaces::msg::GimbalFdb>(
        "gimbal_fdb", rclcpp::SensorDataQoS(),
        std::bind(&Navi_cal::GimbalFdbCallback, this, std::placeholders::_1));
        
        // 初始化参数
        ParamInit();
        
        // 设置参数回调函数，用于动态更新参数
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Navi_cal::paramCallback, this, std::placeholders::_1));
    }

    // 析构函数（空实现）
    Navi_cal::~Navi_cal()
    {
    }

    // 云台反馈消息回调函数
    // 参数：msg - 接收到的云台反馈消息指针
    void Navi_cal::GimbalFdbCallback(const auto_aim_interfaces::msg::GimbalFdb::SharedPtr msg)
    {
        // 保存激光雷达模式和瞄准模式
        lidar_mode = msg->lidar_mode;    // 激光雷达模式（1可能代表低空目标）
        aiming_mode = msg->aiming_mode;  // 瞄准模式（2可能代表低空目标）
    }

    // 弹道补偿计算函数 - 计算俯仰角补偿
    // 参数：
    //   s - 水平距离 (m)
    //   z - 垂直高度差 (m)
    //   v - 弹丸初速度 (m/s)
    // 返回值：补偿后的俯仰角 (度)
    float Navi_cal::pitchTrajectoryCompensation(float s, float z, float v)
    {   
        // 初始化垂直距离和临时变量
        auto dist_vertical = z;
        auto vertical_tmp = dist_vertical;
        // 水平距离
        auto dist_horizonal = s;
        
        // 初始俯仰角（弧度转角度）
        auto pitch = atan(dist_vertical / dist_horizonal) * 180 / PI;
        auto pitch_new = pitch;
        
        // 迭代计算（最多10次），修正弹道带来的俯仰角误差
        for (int i = 0; i < 10; i++)
        {
            auto x = 0.0;  // 累计水平位移
            auto y = 0.0;  // 累计垂直位移
            auto p = tan(pitch_new / 180 * PI);  // 俯仰角的正切值（斜率）
            auto u = v / sqrt(1 + pow(p, 2));    // 水平方向速度分量
            
            // 步长 = 总水平距离 / 迭代次数
            auto delta_x = dist_horizonal / R_K_iter;
            
            // 龙格-库塔法(RK4)迭代计算弹道
            for (int j = 0; j < R_K_iter; j++)
            {
                // K1 - 一阶斜率
                auto k1_u = -k * u * sqrt(1 + pow(p, 2));  // 速度衰减
                auto k1_p = -GRAVITY / pow(u, 2);          // 重力影响
                
                // K2 - 二阶斜率（中点）
                auto k1_u_sum = u + k1_u * (delta_x / 2);
                auto k1_p_sum = p + k1_p * (delta_x / 2);
                auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
                auto k2_p = -GRAVITY / pow(k1_u_sum, 2);
                
                // K3 - 三阶斜率（中点）
                auto k2_u_sum = u + k2_u * (delta_x / 2);
                auto k2_p_sum = p + k2_p * (delta_x / 2);
                auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
                auto k3_p = -GRAVITY / pow(k2_u_sum, 2);
                
                // K4 - 四阶斜率（终点）
                auto k3_u_sum = u + k3_u * delta_x;
                auto k3_p_sum = p + k3_p * delta_x;
                auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
                auto k4_p = -GRAVITY / pow(k3_u_sum, 2);
                
                // RK4公式更新速度和斜率
                u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
                p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
                
                // 更新累计位移
                x += delta_x;
                y += p * delta_x;
            }
            
            // 计算垂直方向误差（期望高度 - 实际高度）
            auto error = dist_vertical - y;
            
            // 误差小于阈值（0.5mm），停止迭代
            if (abs(error) <= 0.0005)
            {
                break;
            }
            else
            {
                // 修正垂直距离，重新计算俯仰角
                vertical_tmp += error;
                pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / PI;
            }
        }
        
        // 返回补偿后的俯仰角
        return pitch_new;
    }

    // 参数初始化函数
    void Navi_cal::ParamInit()
    {
        // 声明参数pit_bias（俯仰角偏置），默认值0.0
        this->declare_parameter<double>("pit_bias", 0.0);
        // 获取参数值，如果参数未设置则使用默认值0.0
        this->get_parameter_or<double>("pit_bias", pit_bias, 0.0);
    }

    // 动态参数更新回调函数
    // 参数：params - 待更新的参数列表
    // 返回值：参数更新结果（成功/失败）
    rcl_interfaces::msg::SetParametersResult Navi_cal::paramCallback(
        const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;  // 默认更新成功
        
        // 遍历参数列表，更新对应的参数
        for (const auto &param : params) {
            if (param.get_name() == "pit_bias") {
                pit_bias = param.as_double();  // 更新俯仰角偏置
                RCLCPP_INFO(this->get_logger(), "参数更新: pit_bias = %.3f", pit_bias);
            }
        }
        return result;
    }

    // 定时器回调函数（25Hz执行）- 主逻辑处理
    void Navi_cal::timerCallback()
    {
        RCLCPP_INFO(this->get_logger(),"timerCallback");
        
        // ========== 第一步：获取激光雷达在map坐标系下的位姿 ==========
        try{
            // 查询map到lidar_link的坐标变换（TimePointZero表示最新的变换）
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","lidar_link",tf2::TimePointZero);
            
            // 保存位置信息
            lidar_pose_.position.x = transform.transform.translation.x;
            lidar_pose_.position.y = transform.transform.translation.y;
            lidar_pose_.position.z = transform.transform.translation.z;
            
            // 将四元数转换为欧拉角（滚转、俯仰、偏航）
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(lidar_roll_,lidar_pitch_,lidar_yaw_);
            
            // 打印激光雷达位姿信息
            RCLCPP_INFO(this->get_logger(),"lidar yaw: %f, lidar x: %f, lidar y: %f,lidar z: %f,",lidar_yaw_,lidar_pose_.position.x,lidar_pose_.position.y,lidar_pose_.position.z);
        }catch(tf2::TransformException  &ex){
            // 捕获TF变换异常并打印错误信息
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }

        // ========== 第二步：获取机器人本体在map坐标系下的位姿 ==========
        try{
            // 查询map到cloud_link的坐标变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","cloud_link",tf2::TimePointZero);
            
            // 保存机器人位置信息
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            robot_pose_.position.z = transform.transform.translation.z;
            
            // 四元数转欧拉角
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(robot_roll_,robot_pitch_,robot_yaw_);
            
            // 打印机器人位姿信息
            RCLCPP_INFO(this->get_logger(),"Current yaw: %f, Current x: %f, Current y: %f,Current z: %f,",robot_yaw_,robot_pose_.position.x,robot_pose_.position.y,robot_pose_.position.z);
        }catch(tf2::TransformException  &ex){ 
            RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }

        // ========== 第三步：计算目标角度和距离 ==========
        // 初始化俯仰角（测试值，后续会被覆盖）
        fdb_msg_.pitch = 2;
        
        // 计算目标在map坐标系下的偏航角（弧度转角度）
        map_yaw_ = atan2((target_y_ - robot_pose_.position.y),(target_x_ - robot_pose_.position.x)) * 180 /3.1415926;
        
        // 计算机器人到目标的水平距离
        target_dis_ = sqrt((target_y_ - robot_pose_.position.y) * (target_y_ - robot_pose_.position.y)+(target_x_ - robot_pose_.position.x) * (target_x_ - robot_pose_.position.x));

        // ========== 第四步：根据模式设置弹道参数 ==========
        if(lidar_mode == 1 || aiming_mode ==2)// 低空目标模式（low）
        { 
            target_z_ = 0.3455;    // 目标高度 0.3455m
            shoot_speed = 15.5;    // 射速 15.5m/s
            // 计算补偿后的俯仰角（加上偏置）
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        else// 高塔目标模式（tower）
        {
            target_z_ = 0.8855;    // 目标高度 0.8855m
            shoot_speed = 15.5;    // 射速 15.5m/s
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }

        // ========== 第五步：特殊区域参数修正 ==========
        // 如果机器人在特定区域内，调整目标高度
        if(robot_pose_.position.x >=-0.660237 && robot_pose_.position.x <= 4.069704 && robot_pose_.position.y >= 5.882196 && robot_pose_.position.y <= 8.184546)
        {
            target_z_ = 0.6855;    // 修正目标高度 0.6855m
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }
        else
        {
            target_z_ = 0.8855;
            shoot_speed = 15.5;
            result_pit_ = (pitchTrajectoryCompensation(target_dis_,target_z_,shoot_speed) + pit_bias);
        }

        // ========== 第六步：计算最终的偏航角和俯仰角 ==========
        // 将机器人偏航角从弧度转换为角度
        robot_yaw_ = robot_yaw_ * 180 /3.1415926;
        // 计算需要转动的偏航角（目标角度 - 机器人当前角度）
        result_yaw_ = map_yaw_ - robot_yaw_; //+往左偏，-往右偏

        // ========== 第七步：填充消息并发布 ==========
        fdb_msg_.yaw = result_yaw_;    // 设置偏航角
        fdb_msg_.pitch = result_pit_;  // 设置俯仰角

        // 打印调试信息
        RCLCPP_INFO(this->get_logger(),"map_yaw_ :%f,",map_yaw_);
        RCLCPP_INFO(this->get_logger(),"robot_yaw_ :%f,",robot_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_yaw_ :%f,",result_yaw_);
        RCLCPP_INFO(this->get_logger(),"result_pit_ :%f,",result_pit_);
        RCLCPP_INFO(this->get_logger(),"result_pit_pid:%f,",result_pit_ * PI / 180);
        RCLCPP_INFO(this->get_logger(),"target_dis_ :%f,",target_dis_);
        RCLCPP_INFO(this->get_logger(),"pit_bias :%f,",pit_bias);  
        RCLCPP_INFO(this->get_logger(),"lidar_mode  :%i,",lidar_mode);  
        RCLCPP_INFO(this->get_logger(),"shoot_speed  :%f,",shoot_speed);  
        RCLCPP_INFO(this->get_logger(),"target_z_ :%f,",target_z_);  

        // 发布反馈消息
        fdb_pub_->publish(fdb_msg_);
    }
}// namespace taurus

// 注册ROS 2组件，使该节点可以被组件管理器加载
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(taurus::Navi_cal);