#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eurobot_msgs/LidarCoord.h>

void LidarCoordCallback(const Eurobot_msgs::LidarCoord::ConstPtr& coord)
{
    ROS_INFO("I heard : [x= %.2f , y= %.2f, theta= %.2f]\n", coord->x , coord->y , coord->theta); 
}

int main(int argc, char **argv)
{

    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "lidar_feedback");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;
    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    ros::Subscriber sub = node.subscribe("Environment", 1000, LidarCoordCallback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}