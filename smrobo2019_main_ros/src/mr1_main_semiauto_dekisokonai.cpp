/*
 * mr1_main_semiauto.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <string>

#include "Coordinates_.hpp"

enum class ControllerStatus : uint16_t
{
    init        = 0x0000,
    shutdown    = 0x0001,

    standby     = 0x0010, // standby at start zone
    sz_to_tp,             // moving from SZ  to TP
    tp_throwing,          // throwing at TP
    tp_to_sz,             // moving from TP to SZ
};

enum class ControllerCommands : uint16_t
{
    shutdown, // shutdown

    standby, // stand-by at SZ

	launcher_set,
    receive_slipper,
//  load_slipper,
    contract, //要るのかは不明 再装填に使える?
    throw_slipper,
//  shrink_cylinder,

    sz_to_tp,
    tp_to_sz,

    set_delay_250ms,
    set_delay_500ms,
    set_delay_1s,

    delay,

    segno,
    dal_segno,

    wait_next_pressed,
};

enum class OpMode : uint8_t
{
    def         = 0b000,
    full_op     = 0b001,

    move_test   = 0b100,
    pickup_test = 0b101,
    throw_test  = 0b110,
    pickup_and_throw_test = 0b111,
};

enum class LauncherStatus : uint16_t
{
    shutdown    = 0x0000,
    reset       = 0x0001,
};

enum class LauncherCommands : uint16_t
{
    shutdown_cmd        = 0b0000,
    reset_cmd           = 0b1001,

    grab_slipper_cmd     = 0b0010,
    throw_cmd           = 0b1000,
};

enum class BaseStatus : uint16_t
{
    shutdown    = 0x0000,
    reset       = 0x0001,
    operational = 0x0010,
};

enum class BaseCommands : uint16_t
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
    operational_cmd = 0x0010,
};

enum class MotorCommands : uint8_t
{
    shutdown_cmd    = 0x0000,
    reset_cmd       = 0x0001,
    homing_cmd      = 0x0010,
};

class Mr1Main
{
public:
    Mr1Main(void);

private:
    void baseStatusCallback(const std_msgs::UInt16::ConstPtr &msg);
    void baseConfCallback(const std_msgs::UInt8::ConstPtr &msg);
    void motorStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
    void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg);
    void control_timer_callback(const ros::TimerEvent &event);

    ros::NodeHandle nh_;

    ros::Subscriber base_status_sub;
    ros::Publisher base_cmd_pub;
    std_msgs::UInt16 base_cmd_msg;

    ros::Subscriber launcher_status_sub;
    ros::Publisher launcher_cmd_pub;
    std_msgs::UInt16 launcher_cmd_msg;

    ros::Subscriber base_conf_sub;

    ros::Subscriber motor_status_sub;
    ros::Publisher load_motor_cmd_pub;
    ros::Publisher load_motor_cmd_pos_pub;
    std_msgs::UInt8 load_motor_cmd_msg;
    std_msgs::Float32 load_motor_cmd_pos_msg;

//    ros::Publisher expand_motor_cmd_pub;
//    ros::Publisher expand_motor_cmd_pos_pub;
//    std_msgs::UInt8 expand_motor_cmd_msg;
//    std_msgs::Float32 expand_motor_cmd_pos_msg;

    ros::Subscriber joy_sub;

    ros::Subscriber goal_reached_sub;
    ros::Publisher target_pub;
    ros::Publisher fine_target_pub;
    ros::Publisher abort_pub;
    ros::Publisher initialpose_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher manual_pub;

    std_msgs::Bool abort_msg;
    std_msgs::Bool manual_msg;
    geometry_msgs::Twist cmd_vel_msg;

    int currentCommandIndex = -1;
    const std::vector<ControllerCommands> *command_list;

    static const std::vector<ControllerCommands> full_op_commands;
    static const std::vector<ControllerCommands> default_commands;
    static const std::vector<ControllerCommands> move_test_commands;
    static const std::vector<ControllerCommands> pickup_test_commands;
    static const std::vector<ControllerCommands> throw_test_commands;
    static const std::vector<ControllerCommands> pickup_and_throw_test_commands;

    OpMode _op_mode;

    int _delay_s = 0;

    ros::Timer control_timer;

    BaseStatus base_last_status = BaseStatus::shutdown;
    LauncherStatus launcher_last_status = LauncherStatus::shutdown;

    void shutdown(void);
    void restart(void);

    void grab_gerege(void);
    void release_gerege(void);
    void grab_slipper(void); //要る
    void release_shagai(void);
    void launcher_set(double angle); //要る
    // void load_slipper(void);
    // void put_arm_back(void);
    // void adjust_arm(void);
    void move_belt(double angle); //要る?
    // void expand(void);
    // void contract(void);
    void throw_slipper(void); //要る
    void shrink_cylinder(void);    

    void set_pose(geometry_msgs::Pose pose);
    void publish_path(nav_msgs::Path path);

    void set_delay(double delay_s);

    // flags
    bool _is_operating = false;
    bool _is_standing_by = false;
    bool _goal_reached = false;
    bool _has_base_restarted = false;
    bool _next_pressed = false;
    bool _abort_pressed = false;
    bool _is_manual_enabled = false;

    inline void clear_flags(void)
    {
        _is_operating = false;
        _is_standing_by = false;
        _goal_reached = false;
        _has_base_restarted = false;
        _next_pressed = false;
        _abort_pressed = false;
        _is_manual_enabled = false;
    }

    static int ButtonA;
	static int ButtonB;
	static int ButtonX;
	static int ButtonY;
	static int ButtonLB;
	static int ButtonRB;
	static int ButtonSelect;
	static int ButtonStart;
	static int ButtonLeftThumb;
	static int ButtonRightThumb;

	static int AxisDPadX;
	static int AxisDPadY;
	static int AxisLeftThumbX;
	static int AxisLeftThumbY;
	static int AxisRightThumbX;
    static int AxisRightThumbY;
    static int AxisLeftTrigger;
    static int AxisRightTrigger;
};

int Mr1Main::ButtonA = 0;
int Mr1Main::ButtonB = 1;
int Mr1Main::ButtonX = 2;
int Mr1Main::ButtonY = 3;
int Mr1Main::ButtonLB = 4;
int Mr1Main::ButtonRB = 5;
int Mr1Main::ButtonSelect = 6;
int Mr1Main::ButtonStart = 7;
int Mr1Main::ButtonLeftThumb = 9;
int Mr1Main::ButtonRightThumb = 10;

int Mr1Main::AxisDPadX = 0;
int Mr1Main::AxisDPadY = 1;
int Mr1Main::AxisLeftThumbX = 6;
int Mr1Main::AxisLeftThumbY = 7;
int Mr1Main::AxisRightThumbX = 3;
int Mr1Main::AxisRightThumbY = 4;
int Mr1Main::AxisLeftTrigger = 2;
int Mr1Main::AxisRightTrigger = 5;

const std::vector<ControllerCommands> Mr1Main::full_op_commands( //多分これをメインで使うよ
    {
        ControllerCommands::standby,

        ControllerCommands::segno,

		ControllerCommands::receive_slipper,

        ControllerCommands::sz_to_tp,

        ControllerCommands::throw_slipper,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::contract,

        ControllerCommands::tp_to_sz,

        ControllerCommands::dal_segno
    }
);

const std::vector<ControllerCommands> Mr1Main::move_test_commands( //後回し
    {
        ControllerCommands::standby,

        ControllerCommands::sz_to_pp,
        ControllerCommands::wait_next_pressed,
        ControllerCommands::pp_to_pp1,
        ControllerCommands::wait_next_pressed,
        ControllerCommands::pp1_to_tp,
        ControllerCommands::wait_next_pressed,
    }
);

const std::vector<ControllerCommands> Mr1Main::default_commands( //後回し
    {
        ControllerCommands::standby,

        ControllerCommands::segno,

        ControllerCommands::sz_to_pp,

        ControllerCommands::grab_slipper,
        ControllerCommands::load_slipper,

        ControllerCommands::wait_next_pressed,
        ControllerCommands::pp1_to_tp,
        ControllerCommands::release_shagai,
        ControllerCommands::adjust_arm,        
        ControllerCommands::expand,
        ControllerCommands::throw_slipper,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::shrink_cylinder,
        ControllerCommands::contract,

        ControllerCommands::put_arm_back,
        ControllerCommands::wait_next_pressed,
        ControllerCommands::tp_to_pp2,
        ControllerCommands::grab_slipper,
        ControllerCommands::load_slipper,

        ControllerCommands::wait_next_pressed,
        ControllerCommands::pp2_to_tp,
        ControllerCommands::release_shagai,
        ControllerCommands::adjust_arm,  
        ControllerCommands::expand,
        ControllerCommands::throw_slipper,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::shrink_cylinder,
        ControllerCommands::contract,
        
        ControllerCommands::put_arm_back,
        ControllerCommands::wait_next_pressed,
        ControllerCommands::tp_to_pp3,
        ControllerCommands::grab_slipper,
        ControllerCommands::load_slipper,

        ControllerCommands::wait_next_pressed,
        ControllerCommands::pp3_to_tp,
        ControllerCommands::release_shagai,
        ControllerCommands::adjust_arm,  
        ControllerCommands::expand,
        ControllerCommands::throw_slipper,
        ControllerCommands::set_delay_1s,
        ControllerCommands::delay,
        ControllerCommands::shrink_cylinder,
        ControllerCommands::contract,

        ControllerCommands::put_arm_back,
        ControllerCommands::wait_next_pressed,
        ControllerCommands::tp_to_sz,

        ControllerCommands::dal_segno
    }
);

const std::vector<ControllerCommands> Mr1Main::pickup_test_commands(
    {
        ControllerCommands::standby,

        ControllerCommands::segno,

        ControllerCommands::grab_slipper,
        ControllerCommands::load_slipper,
        ControllerCommands::release_shagai,
        ControllerCommands::put_arm_back,

        // repeat forever
        ControllerCommands::dal_segno
    }
);

const std::vector<ControllerCommands> Mr1Main::throw_test_commands(
    {
        ControllerCommands::standby,

        ControllerCommands::segno,

        ControllerCommands::expand,
        ControllerCommands::throw_slipper,
        ControllerCommands::shrink_cylinder,
        ControllerCommands::contract,

        // repeat forever
        ControllerCommands::dal_segno
    }
);

const std::vector<ControllerCommands> Mr1Main::pickup_and_throw_test_commands(
    {
        ControllerCommands::standby,

        ControllerCommands::segno,

        ControllerCommands::grab_slipper,
        ControllerCommands::load_slipper,
        ControllerCommands::release_shagai,
        ControllerCommands::adjust_arm,
        ControllerCommands::expand,
        ControllerCommands::throw_slipper,
        ControllerCommands::shrink_cylinder,
        ControllerCommands::contract,
        ControllerCommands::put_arm_back,

        // repeat forever
        ControllerCommands::dal_segno
    }
);

Mr1Main::Mr1Main(void)
{
    this->base_status_sub = nh_.subscribe<std_msgs::UInt16>("base/status", 10, &Mr1Main::baseStatusCallback, this);
    this->base_cmd_pub = nh_.advertise<std_msgs::UInt8>("base/cmd", 10); //UInt16->8

    this->motor_status_sub = nh_.subscribe<std_msgs::UInt8>("motor_status", 10, &Mr1Main::motorStatusCallback, this);

    this->load_motor_cmd_pub = nh_.advertise<std_msgs::UInt8>("load_motor_cmd", 10);
    this->load_motor_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("load_motor_cmd_pos", 10);

    this->expand_motor_cmd_pub = nh_.advertise<std_msgs::UInt8>("expand_motor_cmd", 10);
    this->expand_motor_cmd_pos_pub = nh_.advertise<std_msgs::Float32>("expand_motor_cmd_pos", 10);

    this->launcher_cmd_pub = nh_.advertise<std_msgs::UInt16>("launcher/cmd", 1);

    this->base_conf_sub = nh_.subscribe<std_msgs::UInt8>("base/conf", 10, &Mr1Main::baseConfCallback, this);

    this->joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Mr1Main::joyCallback, this);

    this->goal_reached_sub = nh_.subscribe<std_msgs::Bool>("goal_reached", 10, &Mr1Main::goalReachedCallback, this);
    this->target_pub = nh_.advertise<nav_msgs::Path>("target_path", 1);
    this->fine_target_pub = nh_.advertise<nav_msgs::Path>("fine_target_path", 1);
    this->abort_pub = nh_.advertise<std_msgs::Bool>("abort", 1);
    this->initialpose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    this->cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    this->manual_pub = nh_.advertise<std_msgs::Bool>("manual", 1);

    nh_.getParam("ButtonA", ButtonA);
    nh_.getParam("ButtonB", ButtonB);
    nh_.getParam("ButtonX", ButtonX);
    nh_.getParam("ButtonY", ButtonY);
    nh_.getParam("ButtonLB", ButtonLB);
    nh_.getParam("ButtonRB", ButtonRB);
    nh_.getParam("ButtonSelect", ButtonSelect);
    nh_.getParam("ButtonStart", ButtonStart);
    nh_.getParam("ButtonLeftThumb", ButtonLeftThumb);
    nh_.getParam("ButtonRightThumb", ButtonRightThumb);

    nh_.getParam("AxisLeftThumbX", AxisLeftThumbX);
    nh_.getParam("AxisLeftThumbY", AxisLeftThumbY);
    nh_.getParam("AxisRightThumbX", AxisRightThumbX);
    nh_.getParam("AxisRightThumbY", AxisRightThumbY);

    this->_op_mode = OpMode::def;
    this->command_list = &Mr1Main::default_commands;

    // timer starts immediately
    this->control_timer = nh_.createTimer(ros::Duration(0.05), &Mr1Main::control_timer_callback, this);
}

void Mr1Main::baseStatusCallback(const std_msgs::UInt16::ConstPtr &msg)
{
    BaseStatus status = (BaseStatus)msg->data;

    switch (status)
    {
        case BaseStatus::shutdown:
            if (this->base_last_status != BaseStatus::shutdown)
            {
                this->shutdown();
            }
            break;

        case BaseStatus::reset:
            if (this->base_last_status == BaseStatus::shutdown)
            {
                this->restart();
                this->_has_base_restarted = true;
            }
            break;

        default:
            break;
    }

    base_last_status = status;
}

void Mr1Main::baseConfCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    if (this->currentCommandIndex != -1 && this->currentCommandIndex != 0)
    {
        return;
    }

    if (this->_op_mode != (OpMode)msg->data)
    {
        this->_op_mode = (OpMode)msg->data;

        if (this->_op_mode == OpMode::full_op)
        {
            this->command_list = &Mr1Main::full_op_commands;
            ROS_INFO("operation mode set to full_op.");
        }
        else if (this->_op_mode == OpMode::move_test)
        {
            this->command_list = &Mr1Main::move_test_commands;
            ROS_INFO("operation mode set to move_test.");
        }
        else if (this->_op_mode == OpMode::pickup_test)
        {
            this->command_list = &Mr1Main::pickup_test_commands;
            ROS_INFO("operation mode set to pickup_test.");
        }
        else if (this->_op_mode == OpMode::throw_test)
        {
            this->command_list = &Mr1Main::throw_test_commands;
            ROS_INFO("operation mode set to throw_test.");
        }
        else if (this->_op_mode == OpMode::pickup_and_throw_test)
        {
            this->command_list = &Mr1Main::pickup_and_throw_test_commands;
            ROS_INFO("operation mode set to pickup_and_throw_test.");
        }
    }
}

void Mr1Main::motorStatusCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    BaseStatus status = (BaseStatus)msg->data;

    switch (status)
    {
        case BaseStatus::shutdown:
            if (this->base_last_status != BaseStatus::shutdown)
            {
                this->shutdown();
            }
            break;

        case BaseStatus::reset:
            if (this->base_last_status == BaseStatus::shutdown)
            {
                this->restart();
                this->_has_base_restarted = true;
            }
            break;

        default:
            break;
    }

    base_last_status = status;
}

void Mr1Main::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    static bool last_a = false;
    static bool last_b = false;
    static bool last_x = false;
    static bool last_y = false;

    bool _a = joy->buttons[ButtonA];
    bool _b = joy->buttons[ButtonB];
    bool _x = joy->buttons[ButtonX];
    bool _y = joy->buttons[ButtonY];

    bool _start = joy->buttons[ButtonStart];

    if (_start)
    {
        this->shutdown();
    }

    if (_a && !last_a)
    {
        this->_next_pressed = true;
    }
    else if (_b && !last_b)
    {
        this->_abort_pressed = true;
    }
    else if (_x && !last_x)
    {
    }
    else if (_y && !last_y)
    {
    }

    last_a = _a;
    last_b = _b;
    last_x = _x;
    last_y = _y;

    if (this->_is_manual_enabled)
    {
        double vel_x = joy->axes[AxisRightThumbX];
        double vel_y = joy->axes[AxisRightThumbY];
        double vel_yaw_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw_r = (joy->axes[AxisRightTrigger] - 1.0) * (- 1.0 - 0.0) / (- 1.0 - 1.0) + 0.0;
        double vel_yaw = vel_yaw_l + vel_yaw_r;

        double vel_norm = hypot(vel_x, vel_y);
        if (vel_norm > 1.0)
        {
            vel_x /= vel_norm;
            vel_y /= vel_norm;
        }

        this->cmd_vel_msg.linear.x = -vel_x;
        this->cmd_vel_msg.linear.y = vel_y;
        this->cmd_vel_msg.angular.z = vel_yaw;
        this->cmd_vel_pub.publish(this->cmd_vel_msg);
    }
}

void Mr1Main::goalReachedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (!msg->data)
    {
        // not reached yet
        return;
    }

    abort_msg.data = true;
    this->abort_pub.publish(abort_msg);

    this->_goal_reached = true;
}

void Mr1Main::shutdown(void)
{
    if (this->currentCommandIndex != -1)
    {
        ROS_INFO("aborting.");

        this->currentCommandIndex = -1;
    }

    abort_msg.data = true;
    this->abort_pub.publish(abort_msg);

    launcher_cmd_msg.data = (uint16_t)LauncherCommands::shutdown_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);

    base_cmd_msg.data = (uint16_t)BaseCommands::shutdown_cmd;
    base_cmd_pub.publish(base_cmd_msg);

    load_motor_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    load_motor_cmd_pub.publish(load_motor_cmd_msg);

    expand_motor_cmd_msg.data = (uint8_t)MotorCommands::shutdown_cmd;
    expand_motor_cmd_pub.publish(expand_motor_cmd_msg);
}

void Mr1Main::restart(void)
{
    this->currentCommandIndex = 0;
    ROS_INFO("restarting.");

    launcher_cmd_msg.data = (uint16_t)LauncherCommands::reset_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);

    base_cmd_msg.data = (uint16_t)BaseCommands::reset_cmd;
    base_cmd_pub.publish(base_cmd_msg);

    load_motor_cmd_msg.data = (uint8_t)MotorCommands::reset_cmd;
    load_motor_cmd_pub.publish(load_motor_cmd_msg);

    expand_motor_cmd_msg.data = (uint8_t)MotorCommands::reset_cmd;
    expand_motor_cmd_pub.publish(expand_motor_cmd_msg);

    if (this->_op_mode == OpMode::full_op)
    {
        base_cmd_msg.data = (uint16_t)BaseCommands::operational_cmd;
        base_cmd_pub.publish(base_cmd_msg);
    }

    this->clear_flags();
}

void Mr1Main::set_pose(geometry_msgs::Pose pose)
{
    geometry_msgs::PoseWithCovarianceStamped initialpose_msg;
    // assuming that the robot is at start position
    initialpose_msg.header.frame_id = "map";
    initialpose_msg.header.stamp = ros::Time::now();
    initialpose_msg.pose.pose = pose;
    initialpose_msg.pose.covariance =
        {
            0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.025, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.006853891945200942};

    this->initialpose_pub.publish(initialpose_msg);
}

void Mr1Main::publish_path(nav_msgs::Path path)
{
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    this->set_pose(path.poses.at(0).pose);

    this->target_pub.publish(path);
}

void Mr1Main::control_timer_callback(const ros::TimerEvent &event)
{
    if (this->command_list->size() <= this->currentCommandIndex)
    {
        this->shutdown();

        return;
    }

    if (this->currentCommandIndex == -1)
    {
        this->currentCommandIndex = 0;
    }

    this->manual_msg.data = this->_is_manual_enabled;
    this->manual_pub.publish(this->manual_msg);

    ControllerCommands currentCommand = this->command_list->at(this->currentCommandIndex);

    if (currentCommand == ControllerCommands::shutdown)
    {
        this->shutdown();
    }
    else if (currentCommand == ControllerCommands::standby)
    {
        if (!this->_has_base_restarted)
        {
            return;
        }

        if(this->_is_standing_by)
        {
            if (this->_next_pressed)
            {
                clear_flags();
                this->currentCommandIndex++;
                ROS_INFO("starting.");
            }
        }
        else
        {
            set_pose(Coordinates::GetInstance()->get_sz());
            this->_is_standing_by = true;

            ROS_INFO("standing by.");
        }
    }
    else if (currentCommand == ControllerCommands::sz_to_pp)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : pp");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_sz_to_pp());

            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : sz -> pp");
        }
    }
    else if (currentCommand == ControllerCommands::pp_to_pp1)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : pp1");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_pp_to_pp1());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : pp -> pp1");
        }
    }
    else if (currentCommand == ControllerCommands::pp1_to_tp)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : tp");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_pp1_to_tp());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : pp1 -> tp");
        }
    }
    else if (currentCommand == ControllerCommands::tp_to_pp2)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : pp2");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_tp_to_pp2());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : tp -> pp2");
        }
    }
    else if (currentCommand == ControllerCommands::pp2_to_tp)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : tp");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_pp2_to_tp());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : pp2 -> tp");
        }
    }
    else if (currentCommand == ControllerCommands::tp_to_pp3)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : pp3");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_tp_to_pp3());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : tp -> pp3");
        }
    }
    else if (currentCommand == ControllerCommands::pp3_to_tp)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : tp");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_pp3_to_tp());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : pp3 -> tp");
        }
    }
    else if (currentCommand == ControllerCommands::tp_to_sz)
    {
        if (this->_is_operating)
        {
            if (this->_abort_pressed || this->_goal_reached)
            {
                this->_is_operating = false;
                this->_abort_pressed = false;
                this->_goal_reached = false;
                this->currentCommandIndex++;
                ROS_INFO("goal reached : sz");
            }
        }
        else
        {
            this->publish_path(Coordinates::GetInstance()->get_path_tp_to_sz());
            clear_flags();
            this->_goal_reached = false;
            this->_is_operating = true;

            ROS_INFO("moving : tp -> sz");
        }
    }
    else if (currentCommand == ControllerCommands::release_gerege)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->release_gerege();
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("released the gerege.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::grab_slipper)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->grab_slipper();
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("grabbed the shagai.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::release_shagai)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->release_shagai();
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("released the shagai.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::load_slipper)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->move_arm(2.9);
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("loaded the shagai.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::put_arm_back)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->move_arm(0.15);
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("ready to grab the shagai.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::adjust_arm)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->move_arm(2.0);
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("adjusted arm.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::expand)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->move_belt(-0.005);
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("expanded.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::contract)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->move_belt(-0.51);
                this->_is_manual_enabled = false;
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("contracted.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::throw_slipper)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->throw_slipper();
                this->_is_manual_enabled = false;                
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("throwing the shagai.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::shrink_cylinder)
    {
        if(this->_is_operating)
		{
			if(this->_next_pressed)
			{
                clear_flags();
				this->shrink_cylinder();
                this->_is_manual_enabled = false;                
                this->_is_operating = false;
				this->currentCommandIndex++;
				ROS_INFO("throw completed.");
			}
		}
		else
		{
			clear_flags();
			this->_is_manual_enabled = true;
			this->_is_operating = true;
        }
    }
    else if (currentCommand == ControllerCommands::set_delay_250ms)
    {
        set_delay(0.250);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::set_delay_500ms)
    {
        set_delay(0.500);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::set_delay_1s)
    {
        set_delay(1.000);
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::delay)
    {
        if (this->_delay_s == 0)
        {
            return;
        }

        if (this->_delay_s < ros::Time::now().toSec())
        {
            this->_delay_s = 0;
            this->currentCommandIndex++;
        }
    }
    else if (currentCommand == ControllerCommands::segno)
    {
        this->currentCommandIndex++;
    }
    else if (currentCommand == ControllerCommands::dal_segno)
    {
        auto segno_iter = std::find(this->command_list->begin(), this->command_list->end(), ControllerCommands::segno);
        if (segno_iter == this->command_list->end())
        {
            // abort on error
            this->shutdown();
        }
        auto segno_index = std::distance(this->command_list->begin(), segno_iter);
        this->currentCommandIndex = segno_index;
    }
    else if (currentCommand == ControllerCommands::wait_next_pressed)
    {
		this->_is_manual_enabled = true;
        if (this->_next_pressed)
        {
			this->_is_manual_enabled = false;
            this->currentCommandIndex++;
            this->_next_pressed = false;
        }
    }
}

void Mr1Main::grab_gerege(void)
{
    launcher_cmd_msg.data |= (uint16_t)LauncherCommands::grab_gerege_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::release_gerege(void)
{
    launcher_cmd_msg.data &= ~(uint16_t)LauncherCommands::grab_gerege_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::grab_slipper(void)
{
    launcher_cmd_msg.data |= (uint16_t)LauncherCommands::grab_slipper_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::release_shagai(void)
{
    launcher_cmd_msg.data &= ~(uint16_t)LauncherCommands::grab_slipper_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::move_arm(double angle)
{
    load_motor_cmd_pos_msg.data = angle;
    load_motor_cmd_pos_pub.publish(load_motor_cmd_pos_msg);
}

// void Mr1Main::load_slipper(void)
// {
//     load_motor_cmd_pos_msg.data = M_PI / 2;
//     load_motor_cmd_pos_pub.publish(load_motor_cmd_pos_msg);
// }

// void Mr1Main::put_arm_back(void)
// {
//     load_motor_cmd_pos_msg.data = 0;
//     load_motor_cmd_pos_pub.publish(load_motor_cmd_pos_msg);
// }

// void Mr1Main::adjust_arm(void)
// {
//     load_motor_cmd_pos_msg.data = M_PI / 2;
//     load_motor_cmd_pos_pub.publish(load_motor_cmd_pos_msg);
// }

void Mr1Main::move_belt(double angle)
{
    expand_motor_cmd_pos_msg.data = angle;
    expand_motor_cmd_pos_pub.publish(expand_motor_cmd_pos_msg);
}

// void Mr1Main::expand(void)
// {
//     expand_motor_cmd_pos_msg.data = M_PI;
//     expand_motor_cmd_pos_pub.publish(expand_motor_cmd_pos_msg);
// }

// void Mr1Main::contract(void)
// {
//     expand_motor_cmd_pos_msg.data = 0;
//     expand_motor_cmd_pos_pub.publish(expand_motor_cmd_pos_msg);
// }

void Mr1Main::throw_slipper(void)
{
    launcher_cmd_msg.data ^= (uint16_t)LauncherCommands::throw_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::shrink_cylinder(void)
{
    launcher_cmd_msg.data ^= (uint16_t)LauncherCommands::throw_cmd;
    launcher_cmd_pub.publish(launcher_cmd_msg);
}

void Mr1Main::set_delay(double delay_s)
{
    this->_delay_s = ros::Time::now().toSec() + delay_s;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mr1_main");

    Mr1Main *instance = new Mr1Main();
    ROS_INFO("MR1 main node has started.");

    ros::spin();
    ROS_INFO("MR1 main node has been terminated.");
}
