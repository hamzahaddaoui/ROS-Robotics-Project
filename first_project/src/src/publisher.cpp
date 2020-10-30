#include "ros/ros.h"
#include <limits>
#include "first_project/computeDist.h"
#include "first_project/stat.h"
#include <first_project/parametersConfig.h>
#include <dynamic_reconfigure/server.h>

#define loop_freq 5

int unsafe;
int crash;

void callback(first_project::parametersConfig &config, uint32_t level) {
	unsafe = config.unsafety_threshold;
	crash = config.crash_threshold;
	ROS_INFO("Reconfigure Request: new thresholds ->  <0----CRASH----%d----UNSAFE----%d----SAFE>", crash, unsafe);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "data_publisher");

  	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<first_project::stat>("data_publisher", 10);
	ros::ServiceClient client = n.serviceClient<first_project::computeDist>("compute_distance");
	first_project::computeDist srv;
	first_project::stat msg;

	dynamic_reconfigure::Server<first_project::parametersConfig> server;
  	dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType recfg;

  	recfg = boost::bind(&callback, _1, _2);
  	server.setCallback(recfg);

	ros::Rate loop_rate(loop_freq);  
	
	while (ros::ok()){		
		if (client.call(srv)){
			msg.distance = srv.response.distance;
			if (msg.distance > unsafe)
				msg.flag = "SAFE";
			else if (msg.distance > crash)
				msg.flag = "UNSAFE";
			else
				msg.flag = "CRASH";
				
			chatter_pub.publish(msg);
			ROS_INFO("Distance: %f", msg.distance);
 		}
		else {
			msg.distance = std::numeric_limits<double>::quiet_NaN();
			msg.flag = "NONE";
			chatter_pub.publish(msg);
    		ROS_INFO("Distance not available");
  		}
   		chatter_pub.publish(msg);
    	ros::spinOnce();
    	loop_rate.sleep();
  	}
	
  	return 0;
}
