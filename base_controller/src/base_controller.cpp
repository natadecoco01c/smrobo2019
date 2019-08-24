/*
 * base_controller.cpp
 *
 *  Created on: Dec 23, 2017
 *      Author: yusaku
 *  Modified
 *      modifier: takeyabuyaketa
 */

#include <ros/ros.h>
#include <ros/duration.h> //要るのかこれ?
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <math.h>

class BaseController {
public:
	BaseController();

private:
	void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void TimerCallback(const ros::TimerEvent& event);
	void CalcWheelSpeed(double actualDt);
	void GetCurrentYaw(const std_msgs::Float64::ConstPtr& yaw);

	double MaximumAcceleration;
	double MaximumVelocity;
//	double RobotRadius; //多分要らない
	double wheel_radius;

	int ctrl_freq;

	bool InvertX = false;
	bool InvertY = false;
	bool InvertZ = false;

	bool LimitVelocity = false; //都合が悪いのでなしにした なしに出来てるのかは怪しい
	bool LimitAcceleration = false;

	bool controller_absolute = false;

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

	ros::Subscriber _base_odom_yaw_sub;
	double current_yaw;

	double targetVelX;
	double targetVelY;
	double targetRotZ;

	ros::Time targetTime;

	double lastTarget[4];
	std_msgs::Float32 motorCmdVelmsg[4]; //全てbeta用
	std_msgs::Float32 motorCmdPosmsg[4];

	void calc_wheel(void);
	void calc_VEL_slope(void);
	void calc_VEL_size(void);
	void calc_R(void);
	void calc_turned_VEL(void);
	void calc_X_Y(void);
	void calc_r(void);
	void calc_motor_rad_vel(void); //radとvelとを統合
//	void calc_motor_vel(void);

	double harfPI = M_PI / 2.0;
	double X, Y; //center of rotation
	double x[4] = { 0.519, 0.519, -0.519, -0.519 }, y[4] = { -0.519, 0.519,
			0.519, -0.519 }; //motor position //set paramでやりたいね
	double r[4]; //回転中心からの各タイヤとの距離
	double R; //回転中心と機体中心との距離
	double calced_vel[4]; //XY座標のX+,Y-の位置から左回りに番号
	double rad[4];
	double VEL_slope;
	double VEL_slope2;
	double VEL_size;
	double turned_VEL_x, turned_VEL_y;
};

void BaseController::calc_wheel(void) {
	if ((targetRotZ != 0) && (targetVelX != 0 || targetVelY != 0)) { //回転　並進あり
		calc_VEL_slope();
		calc_VEL_size();
		calc_R();
		calc_turned_VEL();
		calc_X_Y();
		calc_r();
		calc_motor_rad_vel();
//		calc_motor_vel();
	} else if (targetRotZ == 0) { //並進のみ
		calc_VEL_slope();
		calc_VEL_size();
		calc_motor_rad_vel();
//		calc_motor_vel();
	} else {					  //回転のみ
		calc_X_Y();
		calc_r();
		calc_motor_rad_vel();
//		calc_motor_vel();
	}
}

void BaseController::calc_VEL_slope(void) {
	VEL_slope = atan(targetVelY / targetVelX);
	VEL_slope2 = atan2(targetVelY, targetVelX);
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
		lambda = R / sqrt(pow(turned_VEL_x, 2) + pow(turned_VEL_y, 2));
		X = lambda * turned_VEL_x; //角速度が+なら左90°向き -なら右90°
		Y = lambda * turned_VEL_y;
	}
}

void BaseController::calc_r(void) {
	for (int i = 0; i < 4; i++) {
		r[i] = hypot(x[i] - X, y[i] - Y);
	}
}

void BaseController::calc_motor_rad_vel(void) {
	double Rad;
	double Rad2;
	if (targetRotZ != 0) {
		for (int i = 0; i < 4; i++) {
			Rad = atan((x[i] - X) / (-(y[i] - Y))); //XYから見たxy方向のベクトルを左に90°回したベクトルの角度
			Rad2 = atan2(x[i] - X, -(y[i] - Y));
			rad[i] = Rad;
			if (!((Rad2 < harfPI) && (Rad2 > -harfPI))) { //steerは-PI/2からPI/2の間のみ動く　それをはみ出したら速度にマイナス
				calced_vel[i] = -(r[i] * targetRotZ);
			} else {
				calced_vel[i] = r[i] * targetRotZ;
			}
		}
	} else if (VEL_size != 0) {
		if (!((VEL_slope2 < harfPI) && (VEL_slope2 > -harfPI))) //-PI/2からPI/2のとき正回転
		{
			VEL_size *= (-1.0);
		}
		for (int i = 0; i < 4; i++) {
			rad[i] = VEL_slope;
			calced_vel[i] = VEL_size;
		}
	} else {
		for (int i = 0; i < 4; i++) {
			calced_vel[i] = VEL_size;
		}
	}
}
//void BaseController::calc_motor_vel(void) {
//	if (targetRotZ != 0) {
//		for (int i = 0; i < 4; i++) {
//			calced_vel[i] = r[i] * targetRotZ;
//		}
//	} else {
//		if (!((VEL_slope2 <= harfPI) && (VEL_slope2 >= -harfPI))) //-PI/2からPI/2のとき正回転
//		{
//			VEL_size *= (-1.0);
//		}
//		for (int i = 0; i < 4; i++) {
//			calced_vel[i] = VEL_size;
//		}
//	}
//}

BaseController::BaseController(void) {
	auto _nh = ros::NodeHandle("~");

	_nh.param("motor_max_acc", this->MaximumAcceleration, 0.0);
	_nh.param("motor_max_vel", this->MaximumVelocity, 0.0);
//	_nh.param("robot_radius", this->RobotRadius, 0.258);
	_nh.param("wheel_radius", this->wheel_radius, 0.0400);

	_nh.param("controller_absolute", this->controller_absolute, false);

	_nh.param("ctrl_freq", ctrl_freq, 20); //ここ結構重要だと思うんですけど(名推理)

	ROS_INFO("motor_max_acc : %f", this->MaximumAcceleration);
	ROS_INFO("motor_max_vel : %f", this->MaximumVelocity);
//	ROS_INFO("robot_radius : %f", this->RobotRadius);
	ROS_INFO("wheel_radius : %f", this->wheel_radius);
	ROS_INFO("ctrl_freq : %d", this->ctrl_freq);
	ROS_INFO("controller_absolute : %s",
			this->controller_absolute ? "true" : "false");

	if (this->MaximumVelocity < 0) {
		this->LimitVelocity = false;
	}

	if (this->MaximumAcceleration < 0) {
		this->LimitAcceleration = false;
	}

	_nh.param("invert_x", this->InvertX, false);
	_nh.param("invert_y", this->InvertY, false);
	_nh.param("invert_z", this->InvertZ, false);

	_base_odom_yaw_sub = nh.subscribe < std_msgs::Float64
			> ("base/odom/yaw", 10, &BaseController::GetCurrentYaw, this);

	cmdVel_sub = nh.subscribe < geometry_msgs::Twist
			> ("cmd_vel", 10, &BaseController::CmdVelCallback, this);

	motor0CmdVel_pub = nh.advertise < std_msgs::Float32 > ("motor0_cmd_vel", 1); //beta用にfloat32にした
	motor1CmdVel_pub = nh.advertise < std_msgs::Float32 > ("motor1_cmd_vel", 1);
	motor2CmdVel_pub = nh.advertise < std_msgs::Float32 > ("motor2_cmd_vel", 1);
	motor3CmdVel_pub = nh.advertise < std_msgs::Float32 > ("motor3_cmd_vel", 1);

	motor0CmdPos_pub = nh.advertise < std_msgs::Float32 > ("motor0_cmd_pos", 1);
	motor1CmdPos_pub = nh.advertise < std_msgs::Float32 > ("motor1_cmd_pos", 1);
	motor2CmdPos_pub = nh.advertise < std_msgs::Float32 > ("motor2_cmd_pos", 1);
	motor3CmdPos_pub = nh.advertise < std_msgs::Float32 > ("motor3_cmd_pos", 1);

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

void BaseController::GetCurrentYaw(const std_msgs::Float64::ConstPtr& yaw) {
	current_yaw = yaw->data;
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

	if (this->controller_absolute) {
		this->targetVelX = targetVelX * cos(this->current_yaw)
				+ targetVelY * sin(this->current_yaw);
		this->targetVelY = -targetVelX * sin(this->current_yaw)
				+ targetVelY * cos(this->current_yaw);
	}
}

void BaseController::TimerCallback(const ros::TimerEvent& event) {
//	ros::Duration duration(0.00003);
	CalcWheelSpeed(event.current_real.toSec() - event.last_real.toSec());
	motor0CmdVel_pub.publish(motorCmdVelmsg[0]);
//	duration.sleep();
	motor1CmdVel_pub.publish(motorCmdVelmsg[1]);
//	duration.sleep();
	motor2CmdVel_pub.publish(motorCmdVelmsg[2]);
//	duration.sleep();
	motor3CmdVel_pub.publish(motorCmdVelmsg[3]);
//	duration.sleep();
	motor0CmdPos_pub.publish(motorCmdPosmsg[0]);
//	duration.sleep();
	motor1CmdPos_pub.publish(motorCmdPosmsg[1]);
//	duration.sleep();
	motor2CmdPos_pub.publish(motorCmdPosmsg[2]);
//	duration.sleep();
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
		this->motorCmdVelmsg[i].data = -t[i];
		this->motorCmdPosmsg[i].data = -rad[i];
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "base_controller");

	BaseController *baseController = new BaseController();
	ROS_INFO("base_controller node has started.");

	ros::spin();
	ROS_INFO("base_controller node has been terminated.");
}

