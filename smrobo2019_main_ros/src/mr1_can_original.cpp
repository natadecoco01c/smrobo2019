/*
 * mr1_can.cpp
 *
 *  Created on: Feb 27, 2019
 *      Author: yuto
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <boost/array.hpp>

#include <can_msgs/CanFrame.h>

#define CAN_MTU 8

template<typename T>
union _Encapsulator
{
    T data;
    uint64_t i;
};

template <typename T>
void can_unpack(const boost::array<uint8_t, CAN_MTU> &buf, T &data)
{
    _Encapsulator<T> _e;

    for(int i = 0; i < sizeof(T); i++)
    {
        _e.i = (_e.i << 8) | (uint64_t)(buf[i]);
    }

    data = _e.data;
}

template<typename T>
void can_pack(boost::array<uint8_t, CAN_MTU> &buf, const T data)
{
    _Encapsulator<T> _e;
    _e.data = data;

    for(int i = sizeof(T); i > 0;)
    {
        i--;
        buf[i] = _e.i & 0xff;
        _e.i >>= 8;
    }
}

class Mr1CanNode
{
public:
    Mr1CanNode(void);

private:
    void baseCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
    void motor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void motor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);
    void motor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg);

    void launcherCmdCallback(const std_msgs::UInt16::ConstPtr& msg);
    void loadmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void loadmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);    
    void expandmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg);
    void expandmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg);    

    void canRxCallback(const can_msgs::CanFrame::ConstPtr &msg);

    template<typename T>
    void sendData(const uint16_t id, const T data);

    ros::NodeHandle _nh;
    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Publisher  _launcher_status_pub;
    ros::Subscriber _launcher_cmd_sub;

    ros::Publisher	_base_status_pub;
    ros::Publisher  _base_odom_x_pub;
    ros::Publisher  _base_odom_y_pub;
    ros::Publisher  _base_odom_yaw_pub;
    ros::Publisher  _base_conf_pub;

    ros::Subscriber	_base_cmd_sub;
    ros::Subscriber _base_motor0_cmd_vel_sub;
    ros::Subscriber _base_motor1_cmd_vel_sub;
    ros::Subscriber _base_motor2_cmd_vel_sub;

    ros::Publisher  _load_motor_status_pub;
    ros::Subscriber _load_motor_cmd_sub;
    ros::Subscriber _load_motor_cmd_pos_sub;

    ros::Publisher  _expand_motor_status_pub;
    ros::Subscriber _expand_motor_cmd_sub;
    ros::Subscriber _expand_motor_cmd_pos_sub;

    static constexpr uint16_t id_baseStatus             = 0x200;
    static constexpr uint16_t id_baseOdomX              = 0x205;
    static constexpr uint16_t id_baseOdomY              = 0x206;
    static constexpr uint16_t id_baseOdomYaw            = 0x207;
    static constexpr uint16_t id_baseConf               = 0x208;

    static constexpr uint16_t id_launcherStatus	        = 0x300;
    static constexpr uint16_t id_launcherCmd	        = 0x301;

    static constexpr uint16_t id_base_motor0_cmd	    = 0x4a6;
    static constexpr uint16_t id_base_motor0_cmd_vel    = 0x4a7;
    static constexpr uint16_t id_base_motor1_cmd	    = 0x4a2;
    static constexpr uint16_t id_base_motor1_cmd_vel    = 0x4a3;
    static constexpr uint16_t id_base_motor2_cmd	    = 0x4a8;
    static constexpr uint16_t id_base_motor2_cmd_vel    = 0x4a9;

    static constexpr uint16_t id_load_motor_cmd         = 0x4f8; //beta
    static constexpr uint16_t id_load_motor_cmd_pos     = 0x4f9;
    static constexpr uint16_t id_load_motor_status      = 0x4fa;

    static constexpr uint16_t id_expand_motor_cmd         = 0x4f0;
    static constexpr uint16_t id_expand_motor_cmd_pos     = 0x4f1;
    static constexpr uint16_t id_expand_motor_status      = 0x4f3;
};

Mr1CanNode::Mr1CanNode(void)
{
    _can_tx_pub				    = _nh.advertise<can_msgs::CanFrame>("can_tx", 10);
    _can_rx_sub				    = _nh.subscribe<can_msgs::CanFrame>("can_rx", 10, &Mr1CanNode::canRxCallback, this);

    _launcher_status_pub	    = _nh.advertise<std_msgs::UInt16>("launcher/status", 10);
    _launcher_cmd_sub		    = _nh.subscribe<std_msgs::UInt16>("launcher/cmd", 10, &Mr1CanNode::launcherCmdCallback, this);

    _base_status_pub		    = _nh.advertise<std_msgs::UInt16>("base/status", 10);
    _base_odom_x_pub		    = _nh.advertise<std_msgs::Float64>("base/odom/x", 10);
    _base_odom_y_pub		    = _nh.advertise<std_msgs::Float64>("base/odom/y", 10);
    _base_odom_yaw_pub		    = _nh.advertise<std_msgs::Float64>("base/odom/yaw", 10);
    _base_conf_pub			    = _nh.advertise<std_msgs::UInt8>("base/conf", 10);

    _base_cmd_sub			    = _nh.subscribe<std_msgs::UInt16>("base/cmd", 10 , &Mr1CanNode::baseCmdCallback, this);
    _base_motor0_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("base/motor0_cmd_vel", 10, &Mr1CanNode::motor0CmdVelCallback, this);
    _base_motor1_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("base/motor1_cmd_vel", 10, &Mr1CanNode::motor1CmdVelCallback, this);
    _base_motor2_cmd_vel_sub	= _nh.subscribe<std_msgs::Float64>("base/motor2_cmd_vel", 10, &Mr1CanNode::motor2CmdVelCallback, this);

    _load_motor_status_pub      = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _load_motor_cmd_sub	        = _nh.subscribe<std_msgs::UInt8>("load_motor_cmd", 10, &Mr1CanNode::loadmotorCmdCallback, this);
    _load_motor_cmd_pos_sub	    = _nh.subscribe<std_msgs::Float32>("load_motor_cmd_pos", 10, &Mr1CanNode::loadmotorCmdPosCallback, this);

    _expand_motor_status_pub        = _nh.advertise<std_msgs::UInt8>("motor_status", 10);
    _expand_motor_cmd_sub	        = _nh.subscribe<std_msgs::UInt8>("expand_motor_cmd", 10, &Mr1CanNode::expandmotorCmdCallback, this);
    _expand_motor_cmd_pos_sub	    = _nh.subscribe<std_msgs::Float32>("expand_motor_cmd_pos", 10, &Mr1CanNode::expandmotorCmdPosCallback, this);
}


void Mr1CanNode::baseCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    this->sendData(id_base_motor0_cmd, msg->data);
    this->sendData(id_base_motor1_cmd, msg->data);
    this->sendData(id_base_motor2_cmd, msg->data);
}

void Mr1CanNode::motor0CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_base_motor0_cmd_vel, msg->data);
}

void Mr1CanNode::motor1CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_base_motor1_cmd_vel, msg->data);
}

void Mr1CanNode::motor2CmdVelCallback(const std_msgs::Float64::ConstPtr& msg)
{
    this->sendData(id_base_motor2_cmd_vel, msg->data);
}

void Mr1CanNode::launcherCmdCallback(const std_msgs::UInt16::ConstPtr& msg)
{
    this->sendData(id_launcherCmd, msg->data);
}

void Mr1CanNode::loadmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_load_motor_cmd, msg->data);
}

void Mr1CanNode::loadmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_load_motor_cmd_pos, msg->data);
}

void Mr1CanNode::expandmotorCmdCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    this->sendData(id_expand_motor_cmd, msg->data);
}

void Mr1CanNode::expandmotorCmdPosCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->sendData(id_expand_motor_cmd_pos, msg->data);
}

void Mr1CanNode::canRxCallback(const can_msgs::CanFrame::ConstPtr &msg)
{
    std_msgs::UInt16 _launcher_status_msg;
    std_msgs::UInt16 _base_status_msg;
    std_msgs::Float64 _base_odom_x_msg;
    std_msgs::Float64 _base_odom_y_msg;
    std_msgs::Float64 _base_odom_yaw_msg;
    std_msgs::UInt8 _base_conf_msg;
    std_msgs::UInt8 _load_motor_status_msg;
    std_msgs::UInt8 _expand_motor_status_msg;

    switch(msg->id)
    {
        case id_launcherStatus:
            can_unpack(msg->data, _launcher_status_msg.data);
            _launcher_status_pub.publish(_launcher_status_msg);
            break;

        case id_baseStatus:
            can_unpack(msg->data, _base_status_msg.data);
            _base_status_pub.publish(_base_status_msg);
            break;

        case id_baseOdomX:
            can_unpack(msg->data, _base_odom_x_msg.data);
            _base_odom_x_pub.publish(_base_odom_x_msg);
            break;

        case id_baseOdomY:
            can_unpack(msg->data, _base_odom_y_msg.data);
            _base_odom_y_pub.publish(_base_odom_y_msg);
            break;

        case id_baseOdomYaw:
            can_unpack(msg->data, _base_odom_yaw_msg.data);
            _base_odom_yaw_pub.publish(_base_odom_yaw_msg);
            break;

        case id_baseConf:
            can_unpack(msg->data, _base_conf_msg.data);
            _base_conf_pub.publish(_base_conf_msg);
            break;

        case id_load_motor_status:
            can_unpack(msg->data, _load_motor_status_msg.data);
            _load_motor_status_pub.publish(_load_motor_status_msg);
            break;

        case id_expand_motor_status:
            can_unpack(msg->data, _expand_motor_status_msg.data);
            _expand_motor_status_pub.publish(_expand_motor_status_msg);
            break;

        default:
            break;
    }
}

template<typename T>
void Mr1CanNode::sendData(const uint16_t id, const T data)
{
    can_msgs::CanFrame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);

    can_pack<T>(frame.data, data);

    _can_tx_pub.publish(frame);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mr1_can");
    ROS_INFO("mr1_can node has started.");

    Mr1CanNode *mr1CanNode = new Mr1CanNode();

    ros::spin();
}
