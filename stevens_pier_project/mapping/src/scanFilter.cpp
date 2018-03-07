
//This program filters the map using algorithms in Probablist Robotics chapter 6
#include <ros/ros.h>
#include <mapClasses/scanFilter/scanFilter.h>


int main (int argc, char** argv)
{
	std::cout << "this" << std::endl;
	// Initialize ROS
	ros::init (argc, argv, "scanFilterMA");

	ros::NodeHandle nh("~");
	scanFilter sf(&nh);
	ros::spin ();
	std::cout << "this" << std::endl;
	return(0);
}
