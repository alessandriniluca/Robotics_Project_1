#include "ros/ros.h"
#include <sensor_msgs/JointStates.h>
#include <string>

/*
	This class is subscribet to wheel_states. Every time a message arrives, the callback "wheelsCallback" is
	called passing wheelsInfo. It contains all the informations of the wheel. The node Handler will read the 
	bags, and it will convert ticks in RPM.
*/



class pub_sub{
	public:
		pub_sub(ros::NodeHandle n){
			ROS_INFO("Inizio costruttore.");
			n.getParam("/half_length",half_length);
            n.getParam("/half_width",half_width);
            n.getParam("/wheel_radius",wheel_radius);
            n.getParam("/gear_ratio",gear_ratio);
			pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel",1000);
		}


        void wheelsCallback(const sensor_msgs::JointStates &wheelsInfo) {

            geometry_msgs::TwistStamped msg;
            test = wheelsInfo.name;
            msg.name = test
            pub.publish(msg);
        }

	private:
		
		
		ros::Publisher pub;
        std::string test;
		
};


int main(int argc, char** argv){
	ros::init(argc, argv, "handler");
	ROS_INFO("Nodo handler partito.");
	ros::NodeHandle n;
    ros::Subscriber sub_wheels = n.subscribe("/wheel_states", 1000, wheelsCallback);

	ros::spin();
	return 0;
}