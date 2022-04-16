#include <iostream>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <mecanum_wheels/parametersConfig.h>
#include <dynamic_reconfigure/server.h>
#include <cmath>
#define _USE_MATH_DEFINES

/*
	DESCRIPTION

	order of the wheels: front_left front_right rear_left rear_right
*/



class odometry{
	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Publisher pub_odom;
        double ticks[4];
		float half_length;
		float half_width;
		float wheel_radius;
		float gear_ratio;
		int encoder_resolution;
		// Variable needed to discard the first sample
		bool started;
		ros::Time last_time;
		double x; //ahead
		double y; //toward left
		double th; //counter-clockwise
		std::string integration_method;
		tf::TransformBroadcaster broadcaster;

		dynamic_reconfigure::Server<mecanum_wheels::parametersConfig> dynServer;
		dynamic_reconfigure::Server<mecanum_wheels::parametersConfig>::CallbackType f;

		// void param_callback(std::string *int_method, mecanum_wheels::parametersConfig &config, uint32_t level){
		void param_callback(mecanum_wheels::parametersConfig &config, uint32_t level){
			ROS_INFO("Reconfigure Request: %s - Level %d", config.integration_method.c_str(), level);
			
			integration_method = config.integration_method;
		}

		void wheelsCallback(const sensor_msgs::JointState::ConstPtr &wheelsInfo) {
			// initialize the time
			if(!started){
				last_time = wheelsInfo->header.stamp;
				started = true;
				ticks[0] = wheelsInfo->position[0];
				ticks[1] = wheelsInfo->position[1];
				ticks[2] = wheelsInfo->position[2];
				ticks[3] = wheelsInfo->position[3];
				return;
			}

			// Computing delta time for integration
			ros::Time current_time = wheelsInfo->header.stamp;
			double delta = (current_time-last_time).toSec();


			// Computing angular velocities
			// encoder resolution is: #ticks every revolution, need to multiply by 2*pi rad/revolution
			double w1 = (((wheelsInfo->position[0]-ticks[0])/encoder_resolution)*(M_PI*2)/gear_ratio)/delta;
			double w2 = (((wheelsInfo->position[1]-ticks[1])/encoder_resolution)*(M_PI*2)/gear_ratio)/delta;
			double w3 = (((wheelsInfo->position[2]-ticks[2])/encoder_resolution)*(M_PI*2)/gear_ratio)/delta;
			double w4 = (((wheelsInfo->position[3]-ticks[3])/encoder_resolution)*(M_PI*2)/gear_ratio)/delta;

			double vx = (wheel_radius/4) * (w1 + w2 + w3 + w4);
			double vy = (wheel_radius/4) * (-w1 + w2 + w3 -w4);
			double w = (wheel_radius/4) * (1/(half_length+half_width)) * (-w1 +w2 -w3 + w4);

			// publishing message
			geometry_msgs::TwistStamped msg;
			msg.twist.linear.x = vx;
			msg.twist.linear.y = vy;
			msg.twist.linear.z = 0;
			msg.twist.angular.x = 0;
			msg.twist.angular.y = 0;
			msg.twist.angular.z = w;
			msg.header.frame_id = "base_link";
			msg.header.stamp = current_time;

            pub.publish(msg);

			// Computing position
			// integration_method = "RUNGEKUTTA";
			if(integration_method == "EULER"){
				//x += vx*delta*cos(th);
				//y += vy*delta*sin(th);
				//th += w*delta;
				x += (vx*cos(th)-vy*sin(th))*delta;
				y += (vx*sin(th)+vy*cos(th))*delta;
				th += w*delta;
			}else if (integration_method == "RUNGEKUTTA"){
				//x += vx*delta*cos(th+ (w*delta/2));
				//y += vy*delta*sin(th+ (w*delta/2));
				//th += w*delta;
				x += (vx*cos(th+ (w*delta/2))-vy*sin(th+(w*delta/2)))*delta;
				y += (vx*sin(th+ (w*delta/2))+vy*cos(th+(w*delta/2)))*delta;
				th += w*delta;
			}else{
				ROS_ERROR("The selected integration method does not exists. Options: EULER, RUNGEKUTTA");
			}
			
			//publishing
			nav_msgs::Odometry odom_msg;
			odom_msg.pose.pose.position.x = x;
			odom_msg.pose.pose.position.y = y;
			odom_msg.pose.pose.position.z = 0;
			odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
			odom_msg.header.frame_id = "odom";
			odom_msg.child_frame_id = "base_link";
			pub_odom.publish(odom_msg);

			//tf
			geometry_msgs::TransformStamped msg_tf;
			msg_tf.header.stamp = current_time;
			msg_tf.header.frame_id = "odom";
			msg_tf.child_frame_id = "base_link";
			msg_tf.transform.translation.x = x;
			msg_tf.transform.translation.y = y;
			msg_tf.transform.translation.z = 0;
			msg_tf.transform.rotation = tf::createQuaternionMsgFromYaw(th);
			broadcaster.sendTransform(msg_tf);
			

			// Saving new params
			last_time = current_time;
			ticks[0] = wheelsInfo->position[0];
			ticks[1] = wheelsInfo->position[1];
			ticks[2] = wheelsInfo->position[2];
			ticks[3] = wheelsInfo->position[3];
        }

	public:
		odometry(ros::NodeHandle n){
			ROS_INFO("Inizio costruttore.");
			ros::param::get("~half_length",half_length);
            ros::param::get("~half_width",half_width);
            ros::param::get("~wheel_radius",wheel_radius);
            ros::param::get("~gear_ratio",gear_ratio);
			ros::param::get("~encoder_resolution", encoder_resolution);
			f = boost::bind(&odometry::param_callback, this, _1, _2);
			dynServer.setCallback(f);
			pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
			pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1000);
			sub = n.subscribe("/wheel_states", 1000, &odometry::wheelsCallback, this);
			started = false;
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "handler");
	ROS_INFO("Nodo handler partito.");
	ros::NodeHandle n;
	odometry odometry(n);
	ros::spin();
	return 0;
}