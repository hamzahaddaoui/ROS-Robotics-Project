#include "ros/ros.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "first_project/computeDist.h"

#define expireTime 0.5

double updateTime, timeDiff;
double distance;
bool update;


bool compute(first_project::computeDist::Request &req, first_project::computeDist::Response &res) {
	timeDiff = ros::Time::now().toSec() - updateTime;
	if (!update || timeDiff > expireTime){
		update = false;
		ROS_INFO("No updated version of distance.");
		return false;
	}
	
	res.distance = distance;
	ROS_INFO("Distance: %f", distance);
	return true;
}

void callback(const nav_msgs::OdometryConstPtr& msg1, const nav_msgs::OdometryConstPtr& msg2) {
	geometry_msgs::Point car = msg1->pose.pose.position;
	geometry_msgs::Point obs = msg2->pose.pose.position;
	ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", car.x,car.y,car.z, obs.x, obs.y, obs.z);
	
	distance = hypot(hypot(car.x-obs.x, car.y-obs.y), car.z-obs.z);
	ROS_INFO("Distance: %f", distance);
	
	update = true;
	updateTime = ros::Time::now().toSec();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "compute_distance_server");
  	ros::NodeHandle n;

	message_filters::Subscriber<nav_msgs::Odometry> sub1(n, "/car_node/odom", 1);
  	message_filters::Subscriber<nav_msgs::Odometry> sub2(n, "/obs_node/odom", 1);

  	ros::ServiceServer service = n.advertiseService("compute_distance", compute);
  	ROS_INFO("Ready to compute distance");

  	typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> MySyncPolicy;
  
  
  	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
  	sync.registerCallback(boost::bind(&callback, _1, _2));
	ros::spin();
  	
  	return 0;
}
