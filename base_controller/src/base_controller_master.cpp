/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64.h>
#include <math.h>

class BaseController {
public:
	BaseController();

private:
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	void CalcWheelSpeed(double actualDt);

	double MaximumAcceleration;
	double MaximumVelocity;
	double RobotRadius;
	double wheel_radius;

	bool InvertX = false;
	bool InvertY = false;
	bool InvertZ = false;

	bool LimitVelocity = false; //都合が悪いのでなしにした
	bool LimitAcceleration = false;

	ros::NodeHandle nh;

	ros::Subscriber cmdVel_sub;
	ros::Timer control_tim;

	ros::Publisher motor0CmdVel_pub;
	ros::Publisher motor1CmdVel_pub;
	ros::Publisher motor2CmdVel_pub;
	ros::Publisher motor3CmdVel_pub;
	ros::Publisher motor0CmdPos_pub;
	ros::Publisher motor1CmdPos_pub;
	ros::Publisher motor2CmdPos_pub;
	ros::Publisher motor3CmdPos_pub;

	double targetVelX;
	double targetVelY;
	double targetRotZ;

	ros::Time targetTime;

	double lastTarget[4];
	std_msgs::Float64 motorCmdVelmsg[4];
	std_msgs::Float64 motorCmdPosmsg[4];

	void calc_wheel(void);
	void calc_VEL_slope(void);
	void calc_VEL_size(void);
	void calc_R(void);
	void calc_turned_VEL(void);
	void calc_X_Y(void);
	void calc_r(void);
	void calc_motor_rad(void);
	void calc_motor_vel(void);

	double X, Y; //center of rotation
	double x[4] = { 0.519, 0.519, -0.519, -0.519 }, y[4] = { -0.519, 0.519, 0.519, -0.519 }; //motor position //set paramでやりたいね
	double r[4];
	double R;
	//double targetVelX, targetVelY, targetRotZ;
	double calced_vel[4]; //XY座標のX+,Y-の位置から左回りに番号
	double rad[4];
	double VEL_slope;
	double VEL_size;
	double turned_VEL_x, turned_VEL_y;
};

void BaseController::calc_wheel(void){
	if ((targetRotZ != 0) && (targetVelX != 0 || targetVelY != 0)) {
				calc_VEL_slope();
				calc_VEL_size();
				calc_R();
				calc_turned_VEL();
				calc_X_Y();
				calc_r();
				calc_motor_rad();
				calc_motor_vel();
			} else if (targetRotZ == 0) {
				calc_VEL_slope();
				calc_VEL_size();
				calc_motor_rad();
				calc_motor_vel();
			} else {
				calc_X_Y();
				calc_r();
				calc_motor_rad();
				calc_motor_vel();
			}
}

void BaseController::calc_VEL_slope(void) {
	VEL_slope = targetVelY / targetVelX;
}

void BaseController::calc_VEL_size(void) {
	VEL_size = hypot(targetVelX, targetVelY);
}

void BaseController::calc_R(void) {
	R = VEL_size / targetRotZ;
}

void BaseController::calc_turned_VEL(void) {
	turned_VEL_x = -targetVelY;
	turned_VEL_y = targetVelX;
}

void BaseController::calc_X_Y(void) {
	if ((targetVelX == 0) && (targetVelY == 0)) {
		X = 0;
		Y = 0;
	} else {
		double lambda;
		lambda = sqrt(
				pow(R, 2) / (pow(turned_VEL_x, 2) + pow(turned_VEL_y, 2)));
		if (targetRotZ >= 0) {
			X = lambda * turned_VEL_x;
			Y = lambda * turned_VEL_y;
		} else {
			X = (-lambda) * turned_VEL_x;
			Y = (-lambda) * turned_VEL_y;
		}
	}
}

void BaseController::calc_r(void) {
	for (int i = 0; i < 4; i++) {
		r[i] = hypot(x[i] - X, y[i] - Y);
	}
}

void BaseController::calc_motor_rad(void) {
	if (targetRotZ != 0) {
		for (int i = 0; i < 4; i++) {
			double Rad = atan(-1.0 / ((y[i] - Y) / (x[i] - X)));
			if (-6.3 <= Rad && Rad <= 6.3) {
				rad[i] = atan(-1.0 / ((y[i] - Y) / (x[i] - X)));
			}
		}
	} else {
		if (targetVelX != 0 || targetVelY != 0) {
			for (int i = 0; i < 4; i++) {
				rad[i] = atan(VEL_slope);
			}
		} else {
			for (int i = 0; i < 4; i++) {
			}
		}
	}
}
void BaseController::calc_motor_vel(void) {
	if (targetRotZ != 0) {
		for (int i = 0; i < 4; i++) {
			calced_vel[i] = r[i] * targetRotZ;
		}
	} else {
		if ((((targetVelY>=0)&&(targetVelX<0))||((targetVelY<0)&&(targetVelX<0)))) //-PI/2からPI/2のとき正回転
		{
			VEL_size *= (-1.0);
		}
			for (int i = 0; i < 4; i++) {
			calced_vel[i] = VEL_size;
		}
	}
}

BaseController::BaseController(void) {
	auto _nh = ros::NodeHandle("~");

	_nh.param("motor_max_acc", this->MaximumAcceleration, 0.0);
	_nh.param("motor_max_vel", this->MaximumVelocity, 0.0);
	_nh.param("robot_radius", this->RobotRadius, 0.258);
	_nh.param("wheel_radius", this->wheel_radius, 0.0500);

	ROS_INFO("motor_max_acc : %f", this->MaximumAcceleration);
	ROS_INFO("motor_max_vel : %f", this->MaximumVelocity);
	ROS_INFO("robot_radius : %f", this->RobotRadius);

	if (this->MaximumVelocity < 0) {
		this->LimitVelocity = false;
	}

	if (this->MaximumAcceleration < 0) {
		this->LimitAcceleration = false;
	}

	_nh.param("invert_x", this->InvertX, false);
	_nh.param("invert_y", this->InvertY, false);
	_nh.param("invert_z", this->InvertZ, false);

	int ctrl_freq;
	_nh.param("ctrl_freq", ctrl_freq, 20);

	cmdVel_sub = nh.subscribe < geometry_msgs::Twist
			> ("cmd_vel", 10, &BaseController::CmdVelCallback, this);

	motor0CmdVel_pub = nh.advertise < std_msgs::Float64 > ("motor0_cmd_vel", 1);
	motor1CmdVel_pub = nh.advertise < std_msgs::Float64 > ("motor1_cmd_vel", 1);
	motor2CmdVel_pub = nh.advertise < std_msgs::Float64 > ("motor2_cmd_vel", 1);
	motor3CmdVel_pub = nh.advertise < std_msgs::Float64 > ("motor3_cmd_vel", 1);

	motor0CmdPos_pub = nh.advertise < std_msgs::Float64 > ("motor0_cmd_pos", 1);
	motor1CmdPos_pub = nh.advertise < std_msgs::Float64 > ("motor1_cmd_pos", 1);
	motor2CmdPos_pub = nh.advertise < std_msgs::Float64 > ("motor2_cmd_pos", 1);
	motor3CmdPos_pub = nh.advertise < std_msgs::Float64 > ("motor3_cmd_pos", 1);

	control_tim = nh.createTimer(ros::Duration(1.0 / ctrl_freq),
			&BaseController::TimerCallback, this);

	targetVelX = targetVelY = targetRotZ = 0.0;

	lastTarget[0] = 0.0;
	lastTarget[1] = 0.0;
	lastTarget[2] = 0.0;
	lastTarget[3] = 0.0;
	motorCmdVelmsg[0].data = 0.0;
	motorCmdVelmsg[1].data = 0.0;
	motorCmdVelmsg[2].data = 0.0;
	motorCmdVelmsg[3].data = 0.0;
}

void BaseController::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	this->targetVelX = static_cast<double>(msg->linear.x);
	this->targetVelY = static_cast<double>(msg->linear.y);
	this->targetRotZ = static_cast<double>(msg->angular.z);
	this->targetTime = ros::Time::now();

	if (this->InvertX) {
		this->targetVelX *= -1;
	}
	if (this->InvertY) {
		this->targetVelY *= -1;
	}
	if (this->InvertZ) {
		this->targetRotZ *= -1;
	}
}

void BaseController::TimerCallback(const ros::TimerEvent& event) {
	CalcWheelSpeed(event.current_real.toSec() - event.last_real.toSec());
	motor0CmdVel_pub.publish(motorCmdVelmsg[0]);
	motor1CmdVel_pub.publish(motorCmdVelmsg[1]);
	motor2CmdVel_pub.publish(motorCmdVelmsg[2]);
	motor3CmdVel_pub.publish(motorCmdVelmsg[3]);
	motor0CmdPos_pub.publish(motorCmdPosmsg[0]);
	motor1CmdPos_pub.publish(motorCmdPosmsg[1]);
	motor2CmdPos_pub.publish(motorCmdPosmsg[2]);
	motor3CmdPos_pub.publish(motorCmdPosmsg[3]);
}

void BaseController::CalcWheelSpeed(double actualDt) {

	this->calc_wheel();

	double t[4];

	t[0] = calced_vel[0] / wheel_radius;
	t[1] = calced_vel[1] / wheel_radius;
	t[2] = calced_vel[2] / wheel_radius;
	t[3] = calced_vel[3] / wheel_radius;

	double _k = 1.0;

	if (this->LimitVelocity) {
		for (int i = 0; i < 4; i++) {
			auto _a = fabs(t[i]);
			if (_a * _k > this->MaximumVelocity) {
				_k = this->MaximumVelocity / _a;
				ROS_WARN(
						"An infeasible velocity command detected! You might want to look into it.");
			}
		}

		for (int i = 0; i < 4; i++) {
			t[i] *= _k;
		}
	}

	if (this->LimitAcceleration) {
		float maxVelDelta = this->MaximumAcceleration * actualDt;

		_k = 1.0;

		for (int i = 0; i < 4; i++) {
			double diffabs = fabs(t[i] - lastTarget[i]);
			if (diffabs * _k > maxVelDelta) {
				_k = maxVelDelta / diffabs;
				ROS_WARN(
						"An infeasible acceleration detected! You might want to look into it.");
			}
		}

		for (int i = 0; i < 4; i++) {
			t[i] = lastTarget[i] + ((t[i] - lastTarget[i]) * _k);
		}
	}

	for (int i = 0; i < 4; i++) {
		this->lastTarget[i] = t[i];
		this->motorCmdVelmsg[i].data = t[i];
		this->motorCmdPosmsg[i].data = rad[i];
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "base_controller");

	BaseController *baseController = new BaseController();
	ROS_INFO("base_controller node has started.");

	ros::spin();
	ROS_INFO("base_controller node has been terminated.");
}

