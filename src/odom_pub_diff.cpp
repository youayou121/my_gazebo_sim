#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>

#define INIT_DATA 0X3F3F3F3F
#define C5_DISTANCE_WHEELS  0.825
#define C5_DIAMETER_WHEELS  0.24
#define PI 3.1415926
#define EXP_DOUBLE 1e-6
 
//说明：此模块为只融合了编码器和IMU（简单融合）的轮式里程计模块
//未考虑到轮子和打滑和车身飘逸的情况，未能处理机器人绑架问题
 
//在gazebo中的测试效果如下：
//1cmd_vel的线速度为0.5m/s时，直线行走30m，误差在2cm左右
//2cmd_vel的线速度为0.1m/s时，直线行走30m，误差在4mm左右
//gazebo中存在车轮打滑现象，当线速度较慢时，车轮打滑现象会改善
 
 
//一开始认为是数据精度导致的误差所以使用long double
//后续发现double其实也一样，误差是因为在gazebo中车身打滑导致
 
double odom_pose_move_conv[] = {1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e3};
\
double odom_pose_static_conv[] = {1e-9, 0, 0, 0, 0, 0,
                                  0, 1e-3, 1e-9, 0, 0, 0,
                                  0, 0, 1e6, 0, 0, 0,
                                  0, 0, 0, 1e6, 0, 0,
                                  0, 0, 0, 0, 1e6, 0,
                                  0, 0, 0, 0, 0, 1e-9};
 
double odom_twist_move_conv[] = {1e-3, 0, 0, 0, 0, 0,
                                 0, 1e-3, 0, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e3};
 
double  odom_twist_static_conv[] = {1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9};
class RobotStatus
{
    public:
        //通过/joint_state话题能够得到两个驱动轮的角度位置
        //此数据相当于编码器的作用
        long double last_right_wheel_position_ = INIT_DATA;
        long double last_left_wheel_position_ = INIT_DATA;
 
 
        long double current_right_wheel_position_;
        long double current_left_wheel_position_;
 
        //当前车轮的角速度
        long double current_right_wheel_angular_;
        long double current_left_wheel_angular_;
 
        //当前车轮的线速度 = 车轮角速度x半径
        long double current_right_wheel_linear_;
        long double current_left_wheel_linear_;
 
        //记录时间
        ros::Time last_time_;
        ros::Time current_time_;
 
        //轮间距和车轮直径
        double distance_wheels_ = C5_DISTANCE_WHEELS;
        double diameter_wheels_ = C5_DIAMETER_WHEELS;
 
        double current_yaw_ = 0.0;
 
        long double robot_liner_x_= 0.0;
        long double robot_liner_y_= 0.0;
 
 
        long double position_x_ = 0.0;
        long double position_y_ = 0.0;
        long double position_z_ = 0.0;
 
        long double robot_angular_yaw_= 0.0;
        tf::Quaternion quat;
}robot_status;
 
//条件变量，每次更新数据后至为true
bool flag_publish_odom = false;
 
void JointStateCallBack(const sensor_msgs::JointState& msg)
{
 
    double dt = 0;
    //判断是否刚刚运行里程计模块
    if(robot_status.last_right_wheel_position_ == INIT_DATA && robot_status.last_left_wheel_position_ == INIT_DATA)
    {
        robot_status.current_right_wheel_angular_ = 0;
        robot_status.current_left_wheel_angular_ = 0;
        robot_status.last_right_wheel_position_ = msg.position[0]; //第一个数据是右轮
        robot_status.last_left_wheel_position_ = msg.position[1];    //第二个数据是左轮
        robot_status.last_time_ = msg.header.stamp;
    }
    else
    {
        robot_status.current_time_ = msg.header.stamp;
        dt = (robot_status.current_time_ - robot_status.last_time_).toSec();
        if(dt < 0.0001)
        {
            return;
        }
 
        robot_status.current_right_wheel_position_ = msg.position[0];
        robot_status.current_left_wheel_position_ = msg.position[1];
 
        //通过得到相对于上一时刻轮子的偏移量，除间隔时间，获得车轮角速度
        robot_status.current_right_wheel_angular_ = (robot_status.current_right_wheel_position_ - robot_status.last_right_wheel_position_) / dt;
        robot_status.current_left_wheel_angular_ = (robot_status.current_left_wheel_position_ - robot_status.last_left_wheel_position_) / dt;
 
        //车轮角速度乘以半径获得线速度
        robot_status.current_right_wheel_linear_ = robot_status.current_right_wheel_angular_ * robot_status.diameter_wheels_ / 2.0;
        robot_status.current_left_wheel_linear_ = robot_status.current_left_wheel_angular_ * robot_status.diameter_wheels_ / 2.0;
 
        //根据差速轮的运动学模型算出机器人的线速度，而机器人的角速度不通过运算获取，而是直接读取IMU数据获得
        robot_status.robot_liner_x_ = (robot_status.current_right_wheel_linear_ + robot_status.current_left_wheel_linear_) / 2.0;
 
        //双差速轮模型
        //推算出偏移量，这里应该要考虑到robot_liner_y，但目前假设没有任何打滑和漂移现象，robot_liner_y_为0
        long double delta_x = (robot_status.robot_liner_x_ * cos(robot_status.current_yaw_) - robot_status.robot_liner_y_*sin(robot_status.current_yaw_))*dt;
        long double delta_y = (robot_status.robot_liner_x_ * sin(robot_status.current_yaw_) + robot_status.robot_liner_y_*cos(robot_status.current_yaw_))*dt;
 
        robot_status.position_x_ += delta_x;
        robot_status.position_y_ += delta_y;
 
 
        robot_status.last_right_wheel_position_ = robot_status.current_right_wheel_position_;
        robot_status.last_left_wheel_position_ = robot_status.current_left_wheel_position_;
        robot_status.last_time_ = robot_status.current_time_;
 
        std::cout << dt << "当前机器人线速度：" << robot_status.robot_liner_x_ << std::endl;
 
        //更改条件变量，准备发布里程计数据
        flag_publish_odom = true;
 
    }
 
}
 
 
void ImuCallBack(const sensor_msgs::Imu & msg)
{
    //机器人的偏航角度和z轴的角速度不通过差速轮的运动学推算，直接通过IMU获取。
    // std::cout<<msg.orientation.w<<std::endl;
    tf::quaternionMsgToTF(msg.orientation, robot_status.quat);
    
    double roll, pitch, yaw;
    tf::Matrix3x3(robot_status.quat).getRPY(roll, pitch, yaw);
    if(flag_publish_odom == false)
    {
        robot_status.current_yaw_ = yaw;
        robot_status.robot_angular_yaw_ = msg.angular_velocity.z;
    }
}
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wheel_odom_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Subscriber joint_state_sub = nh.subscribe("/tracer/joint_states", 60, JointStateCallBack);
  ros::Subscriber imu_sub = nh.subscribe("/imu" , 60 , ImuCallBack);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped base2odom;
  base2odom.child_frame_id = "base_link";
  base2odom.header.frame_id = "odom";  
  robot_status.last_time_ = ros::Time::now();
  robot_status.current_time_ = ros::Time::now();
  while(ros::ok())
  {
 
      if(flag_publish_odom == true)
      {
 
          geometry_msgs::Quaternion odom_quat;
          odom_quat.w = robot_status.quat.getW();
          odom_quat.x = robot_status.quat.getX();
          odom_quat.y = robot_status.quat.getY();
          odom_quat.z = robot_status.quat.getZ();
 
          nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time::now();
          odom.header.frame_id = "odom";
          odom.child_frame_id = "base_link";
 
          odom.pose.pose.position.x = robot_status.position_x_;
          odom.pose.pose.position.y = robot_status.position_y_;
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = odom_quat;
 
          odom.child_frame_id = "base_link";
          odom.twist.twist.linear.x = robot_status.robot_liner_x_;
          odom.twist.twist.angular.z = robot_status.robot_angular_yaw_;
 
          //加入协方差用于扩展卡尔曼滤波
          if(fabs(robot_status.robot_liner_x_) > EXP_DOUBLE || fabs(robot_status.robot_angular_yaw_) > EXP_DOUBLE)
          {
              for(int i = 0 ; i < 36; i++)
              {
                  odom.pose.covariance.at(i) = odom_pose_move_conv[i];
                  odom.twist.covariance.at(i) = odom_twist_move_conv[i];
              }
          }
          else
          {
              for(int i = 0 ; i < 36; i++)
              {
                  odom.pose.covariance.at(i) = odom_pose_static_conv[i];
                  odom.twist.covariance.at(i) = odom_twist_static_conv[i];
              }
          }
          base2odom.transform.rotation = odom.pose.pose.orientation;
          base2odom.transform.translation.x = odom.pose.pose.position.x;
          base2odom.transform.translation.y = odom.pose.pose.position.y;
          base2odom.transform.translation.z = odom.pose.pose.position.z;
          base2odom.header.stamp = ros::Time::now();
          //发布里程计数据
          odom_broadcaster.sendTransform(base2odom);
          odom_pub.publish(odom);
          flag_publish_odom = false;
      }
  }
  ros::shutdown();
  return 0;
}