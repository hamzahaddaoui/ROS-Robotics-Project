#include "ros/ros.h"
#include <limits>
#include "first_project/stat.h"
#include "first_project/computeDist.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "add_two_ints_client");

  	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<first_project::stat>("data_publisher", 10);
	ros::ServiceClient client = n.serviceClient<first_project::computeDist>("compute_distance");
	first_project::computeDist srv;
	first_project::stat msg;
	ros::Rate loop_rate(10);
	
	while (ros::ok()){		
		msg = NULL;	
		if (client.call(srv)){
			msg.distance = srv.response.distance;
			msg.flag = "SAFE";
			chatter_pub.publish(msg);
			ROS_INFO("Distance: %f", msg.distance);
 		}
		else {
			//msg.distance = std::numeric_limits<double>::quiet_NaN();
			chatter_pub.publish(msg);
    		ROS_WARN("Distance not available!");
    		return 1;
  		}
   		chatter_pub.publish(msg);
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
	
  	return 0;
}
