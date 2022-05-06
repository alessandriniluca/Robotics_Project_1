#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include "mecanum_wheels/controlMessage.h"
#include <string>

/*
	This class is subscribet to wheel_states. Every time a message arrives, the callback "wheelsCallback" is
	called passing wheelsInfo. It contains all the informations of the wheel. The node Handler will read the 
	bags, and it will convert ticks in RPM.
*/



class wheelsControl{
	private:
		ros::Publisher pub;
		ros::Subscriber sub;
        std::vector<std::__cxx11::basic_string<char> > test;
		float half_length;
		float half_width;
		float wheel_radius;
		float gear_ratio;
        double w1;
        double w2;
        double w3;
        double w4;

		void wheelsControlCallback(const geometry_msgs::TwistStamped::ConstPtr &velInfo) {            
            double vx = velInfo->twist.linear.x;
            double vy = velInfo->twist.linear.y;
            double w = velInfo->twist.angular.z;

            w1 = (((vx - vy - (half_length+half_width)*w)/wheel_radius)*gear_ratio)*60;
            w2 = (((vx + vy + (half_length+half_width)*w)/wheel_radius)*gear_ratio)*60;
            w3 = (((vx + vy - (half_length+half_width)*w)/wheel_radius)*gear_ratio)*60;
            w4 = (((vx - vy + (half_length+half_width)*w)/wheel_radius)*gear_ratio)*60;

            mecanum_wheels::controlMessage msg;
            msg.rpm_fl = w1;
            msg.rpm_fr = w2;
            msg.rpm_rr = w4;
            msg.rpm_rl = w3;
            msg.header.frame_id = velInfo->header.frame_id;
			msg.header.stamp = velInfo->header.stamp;

            pub.publish(msg);
        }

	public:
		wheelsControl(ros::NodeHandle n){
            ROS_INFO("Inizio costruttore.");
			ros::param::get("~half_length",half_length);
            ros::param::get("~half_width",half_width);
            ros::param::get("~wheel_radius",wheel_radius);
            ros::param::get("~gear_ratio",gear_ratio);
			pub = n.advertise<mecanum_wheels::controlMessage>("wheels_rpm",1000);
			sub = n.subscribe("/cmd_vel", 1000, &wheelsControl::wheelsControlCallback, this);
		}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "handler");
	ROS_INFO("wheelsControl node started.");
	ros::NodeHandle n;
	wheelsControl handler(n);

	ros::spin();
	return 0;
}