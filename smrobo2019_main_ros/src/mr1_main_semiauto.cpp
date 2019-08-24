/*
 * mr1_main_semiauto.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 *      Modifier: takeyabuyaketa
 */

#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
//#include <vector>
#include <string>

enum class fullOP
	: uint8_t
	{
		standby, reload, sz_to_tp, release_reloader, launch, tp_to_sz, restart,
};

enum class BaseStatus
	: uint8_t
	{
		shutdown = 0x00, restart = 0x01,
};
enum class BaseCommands
	: uint8_t
	{
		disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherReloaderCommands
	: uint8_t
	{
		disable = 0x00, enable = 0x01, homing = 0x10,
};

enum class LauncherTriggerCommands
	: uint8_t
	{
		disable = 0x00, enable = 0x01,
};

class Mr1Main {
public:
	Mr1Main(void);

private:
	void baseStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void baseConfCallback(const std_msgs::UInt8::ConstPtr &msg);
	void LauncherReloaderStatusCallback(const std_msgs::UInt8::ConstPtr &msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);
//    void goalReachedCallback(const std_msgs::Bool::ConstPtr &msg);
	void control_timer_callback(const ros::TimerEvent &event);
	void slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg);

	ros::NodeHandle nh_;

	ros::Subscriber base_status_sub;
	ros::Publisher base_cmd_pub;
	std_msgs::UInt8 base_cmd_msg; //launchした時のエラーの原因はおそらくここ 後で下も含めて書き換える時直す

	ros::Subscriber launcher_reloader_status_sub; //ランチャー下げる奴のstatus
	ros::Publisher launcher_cmd_pub; //cmdはbaseと分けました^^
	std_msgs::UInt8 launcher_cmd_msg;

	ros::Publisher launcher_reloader_cmd_pos_pub;
	std_msgs::Float32 launcher_reloader_cmd_pos_msg;

	ros::Publisher launcher_trigger_pub; //トリガー
	std_msgs::UInt8 launcher_trigger_msg;

	ros::Subscriber base_conf_sub;

	ros::Subscriber joy_sub;

	ros::Subscriber slipper_loaded_sub; //スリッパ検出かつスタンバイからのスタート

//  ros::Subscriber goal_reached_sub;
	ros::Publisher target_pub;
	ros::Publisher fine_target_pub;
	ros::Publisher abort_pub;
	ros::Publisher initialpose_pub;
	ros::Publisher cmd_vel_pub;
	ros::Publisher manual_pub;

	std_msgs::Bool abort_msg;
	std_msgs::Bool manual_msg;
	geometry_msgs::Twist cmd_vel_msg;

//	OpMode _op_mode;

	int _delay_s = 0;

	ros::Timer control_timer;

	void shutdown(void); //要らないと思う
	void start(void);
	void restart(void);

	void reloader(float);
	void move_sz_tp(void);
	void move_tp_sz(void);
	void trigger(uint8_t);

	void set_delay(double delay_s);

	fullOP now_command = fullOP::standby; //standby

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

	bool _reload = false;
	bool _release = false;
	bool _trigger = false;

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

Mr1Main::Mr1Main(void) {
	this->base_status_sub = nh_.subscribe < std_msgs::UInt8
			> ("base/status", 10, &Mr1Main::baseStatusCallback, this);
	this->base_cmd_pub = nh_.advertise < std_msgs::UInt8 > ("base/cmd", 10); //UInt16->8

	this->launcher_reloader_status_sub =
			nh_.subscribe < std_msgs::UInt8
					> ("launcher_reloader_status", 10, &Mr1Main::LauncherReloaderStatusCallback, this);

	this->slipper_loaded_sub = nh_.subscribe < std_msgs::UInt8
			> ("slipper_loaded", 10, &Mr1Main::slipper_loaded_callback, this);

	this->launcher_cmd_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher/cmd", 10);
	this->launcher_reloader_cmd_pos_pub = nh_.advertise < std_msgs::Float32
			> ("launcher_reloader_cmd_pos", 10);
	this->launcher_trigger_pub = nh_.advertise < std_msgs::UInt8
			> ("launcher_trigger", 10);

	this->base_conf_sub = nh_.subscribe < std_msgs::UInt8
			> ("base/conf", 10, &Mr1Main::baseConfCallback, this); //これ要る?

	this->joy_sub = nh_.subscribe < sensor_msgs::Joy
			> ("joy", 10, &Mr1Main::joyCallback, this);

//	this->target_pub = nh_.advertise < nav_msgs::Path > ("target_path", 1);
//	this->fine_target_pub = nh_.advertise < nav_msgs::Path
//			> ("fine_target_path", 1);
//	this->abort_pub = nh_.advertise < std_msgs::Bool > ("abort", 1);
//	this->initialpose_pub = nh_.advertise
//			< geometry_msgs::PoseWithCovarianceStamped > ("/initialpose", 1);
	this->cmd_vel_pub = nh_.advertise < geometry_msgs::Twist > ("cmd_vel", 1);
	this->manual_pub = nh_.advertise < std_msgs::Bool > ("manual", 1);

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

	// timer starts immediately
	this->control_timer = nh_.createTimer(ros::Duration(0.05),
			&Mr1Main::control_timer_callback, this); //20Hz?
}

void Mr1Main::baseStatusCallback(const std_msgs::UInt8::ConstPtr &msg) {
	if ((BaseStatus) msg->data == BaseStatus::restart) {
		this->restart();
	}

}

void Mr1Main::baseConfCallback(const std_msgs::UInt8::ConstPtr &msg) //雛形は残しときたい
		{
//	if (this->currentCommandIndex != -1 && this->currentCommandIndex != 0) {
//		return;
//	}
//
//	if (this->_op_mode != (OpMode) msg->data) {
//		this->_op_mode = (OpMode) msg->data;
//
//		if (this->_op_mode == OpMode::full_op) {
//			this->command_list = &Mr1Main::full_op_commands;
//			ROS_INFO("operation mode set to full_op.");
//		} else if (this->_op_mode == OpMode::move_test) {
//			this->command_list = &Mr1Main::move_test_commands;
//			ROS_INFO("operation mode set to move_test.");
//		} else if (this->_op_mode == OpMode::pickup_test) {
//			this->command_list = &Mr1Main::pickup_test_commands;
//			ROS_INFO("operation mode set to pickup_test.");
//		} else if (this->_op_mode == OpMode::throw_test) {
//			this->command_list = &Mr1Main::throw_test_commands;
//			ROS_INFO("operation mode set to throw_test.");
//		} else if (this->_op_mode == OpMode::pickup_and_throw_test) {
//			this->command_list = &Mr1Main::pickup_and_throw_test_commands;
//			ROS_INFO("operation mode set to pickup_and_throw_test.");
//		}
//	}
}

void Mr1Main::LauncherReloaderStatusCallback(
		const std_msgs::UInt8::ConstPtr &msg) {

}

void Mr1Main::slipper_loaded_callback(const std_msgs::UInt8::ConstPtr &msg) {

}

void Mr1Main::control_timer_callback(const ros::TimerEvent &event) {
	if ((fullOP) this->now_command == fullOP::standby) {

	} else if ((fullOP) this->now_command == fullOP::reload) {
		this->reloader(3.3); //下げる
		//now_command ++; //この書き方だとダメだった
	} else if ((fullOP) this->now_command == fullOP::sz_to_tp) {
		//なんか動かす奴
	} else if ((fullOP) this->now_command == fullOP::release_reloader) {
		this->reloader(0.0); //発射の邪魔なので戻す
	} else if ((fullOP) this->now_command == fullOP::launch) {
		this->trigger(0x01);
	} else if ((fullOP) this->now_command == fullOP::tp_to_sz) {
		//なんか動かす奴
	} else if ((fullOP) this->now_command == fullOP::restart) {
		this->restart();
	}
}

void Mr1Main::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
	static bool last_a = false;
	static bool last_b = false;
	static bool last_x = false;
	static bool last_y = false;
	static bool last_start = false;

	bool _a = joy->buttons[ButtonA];
	bool _b = joy->buttons[ButtonB];
	bool _x = joy->buttons[ButtonX];
	bool _y = joy->buttons[ButtonY];

	bool _start = joy->buttons[ButtonStart];

//	if (_start) {
//		this->shutdown();
//	}
	if (_a && !last_a) {
		ROS_INFO("a");
		this->reloader(-11.4);
	} else if (_b && !last_b) {
		ROS_INFO("b");
		this->reloader(0.0);
	} else if (_x && !last_x) {
		this->trigger(0x01);
	} else if (_y && !last_y) {
	} else if (_start && !last_start) {
		ros::Duration duration(0.00003);
		base_cmd_msg.data = (uint8_t) BaseCommands::enable;
		base_cmd_pub.publish(base_cmd_msg);
		duration.sleep();
		launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::enable;
		launcher_cmd_pub.publish(launcher_cmd_msg);
	}

	last_a = _a;
	last_b = _b;
	last_x = _x;
	last_y = _y;

//	if (this->_is_manual_enabled) {
//		double vel_x = joy->axes[AxisRightThumbX];
//		double vel_y = joy->axes[AxisRightThumbY];
//		double vel_yaw_l = (joy->axes[AxisLeftTrigger] - 1.0) * (1.0 - 0.0)
//				/ (-1.0 - 1.0) + 0.0;
//		double vel_yaw_r = (joy->axes[AxisRightTrigger] - 1.0) * (-1.0 - 0.0)
//				/ (-1.0 - 1.0) + 0.0;
//		double vel_yaw = vel_yaw_l + vel_yaw_r;
//
//		double vel_norm = hypot(vel_x, vel_y);
//		if (vel_norm > 1.0) {
//			vel_x /= vel_norm;
//			vel_y /= vel_norm;
//		}
//
//		this->cmd_vel_msg.linear.x = -vel_x;
//		this->cmd_vel_msg.linear.y = vel_y;
//		this->cmd_vel_msg.angular.z = vel_yaw;
//		this->cmd_vel_pub.publish(this->cmd_vel_msg);
//	}
}

void Mr1Main::start(void) {
	ROS_INFO("slipper detected.");
	ros::Duration duration(0.00003);
	base_cmd_msg.data = ((uint8_t) BaseCommands::enable);
	base_cmd_pub.publish(base_cmd_msg);
	duration.sleep();
	launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::enable;
	launcher_cmd_pub.publish(launcher_cmd_msg);
	//now_command = fullOP::
}

void Mr1Main::restart(void) {
	ROS_INFO("restart signal detected.");
	ros::Duration duration(0.00003);
	base_cmd_msg.data = (uint8_t) BaseCommands::disable;
	base_cmd_pub.publish(base_cmd_msg);
	duration.sleep();
	launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::disable;
	launcher_cmd_pub.publish(launcher_cmd_msg);
	duration.sleep();
	base_cmd_msg.data = (uint8_t) BaseCommands::homing;
	base_cmd_pub.publish(base_cmd_msg);
	duration.sleep();
	launcher_cmd_msg.data = (uint8_t) LauncherReloaderCommands::homing;
	launcher_cmd_pub.publish(launcher_cmd_msg);
	duration.sleep();
	launcher_trigger_msg.data = (uint8_t) LauncherTriggerCommands::disable;
	launcher_trigger_pub.publish(launcher_trigger_msg);
	now_command = fullOP::standby;
}

void Mr1Main::reloader(float data) {
	launcher_reloader_cmd_pos_msg.data = data;
	launcher_reloader_cmd_pos_pub.publish(launcher_reloader_cmd_pos_msg);
}

void Mr1Main::trigger(uint8_t data) {
	launcher_trigger_msg.data = data;
	launcher_trigger_pub.publish(launcher_trigger_msg);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mr1_main");

	Mr1Main *instance = new Mr1Main();
	ROS_INFO("MR1 main node has started.");

	ros::spin();
	ROS_INFO("MR1 main node has been terminated.");
}
