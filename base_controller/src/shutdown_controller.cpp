/*
 * shutdown_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

// TODO: dynamically subscribe to various shutdown signal sources specified by parameters

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>


class ShutdownController
{
public:
	ShutdownController();

private:
	void BaseShutdownCallback(const std_msgs::Bool::ConstPtr& msg);
	//void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void TimerCallback(const ros::TimerEvent& event);

	ros::NodeHandle nh;

	ros::Subscriber baseShutdown_sub;

	ros::Publisher shutdown_pub;

	ros::Timer pub_tim;

	std_msgs::Bool shutdown_msg;
};

ShutdownController::ShutdownController(void)
{
	baseShutdown_sub = nh.subscribe<std_msgs::Bool>("base/shutdown", 10, &ShutdownController::BaseShutdownCallback, this);

	shutdown_pub = nh.advertise<std_msgs::Bool>("shutdown", 1);

	pub_tim = nh.createTimer(ros::Duration(0.1), &ShutdownController::TimerCallback, this);

	shutdown_msg.data = true;
}

void ShutdownController::BaseShutdownCallback(const std_msgs::Bool::ConstPtr& msg)
{
	this->shutdown_msg.data = msg->data;
}

void ShutdownController::TimerCallback(const ros::TimerEvent& event)
{
	this->shutdown_pub.publish(this->shutdown_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "shutdown_controller");

	ShutdownController *sdController = new ShutdownController();
	ROS_INFO("shutdown_controller node has started.");

	ros::spin();
	ROS_INFO("shutdown_controller node has been terminated.");
}





