#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/NavSatFix.h"
#include <math.h> 
#include <string> 

class pub_sub {
	private:
    	ros::NodeHandle n;
    	ros::Subscriber sub;
    	tf::TransformBroadcaster tf_pub; //to publish tf
    	ros::Publisher odom_pub;
		nav_msgs::Odometry odom;
		tf::Transform transform;
        float latitude_init, longitude_init, h0;
   		float latitude, longitude, h;
   		
   		float lamb, phi, s, N;
   		float sin_lambda, cos_lambda, sin_phi, cos_phi;
   	
   		float x0, y0, z0;
   		float x, y, z;
   		float xd, yd, zd;
   		float xEast, yNorth, zUp;
   		
   		double const a = 6378137;
        double const b = 6356752.3142;
        double const f = (a - b) / a;
        double const e_sq = f * (2-f);
        float  const deg_to_rad = 0.0174533;

       	

	public:
    		pub_sub(){
        		sub = n.subscribe("/topic", 1000, &pub_sub::callback, this);
       	 		odom_pub = n.advertise<nav_msgs::Odometry>(ros::this_node::getName()+"/odom", 1000);
				n.getParam("/latitude_init", latitude_init);
				n.getParam("/longitude_init", longitude_init);
				n.getParam("/h0", h0);
				
				lamb = deg_to_rad*(latitude_init);
        		phi = deg_to_rad*(longitude_init);
        		s = sin(lamb);
        		N = a / sqrt(1 - e_sq * s * s);

        		sin_lambda = sin(lamb);
        		cos_lambda = cos(lamb);
        		sin_phi = sin(phi);
        		cos_phi = cos(phi);

        		x0 = (h0 + N) * cos_lambda * cos_phi;
        		y0 = (h0 + N) * cos_lambda * sin_phi;
        		z0 = (h0 + (1 - e_sq) * N) * sin_lambda;
    		}

    		void callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
				ROS_INFO_STREAM(ros::this_node::getName());

        		ROS_INFO("Input position: [%f,%f, %f]", msg->latitude, msg->longitude,msg->altitude);
			
				if (msg->latitude == 0 && msg->longitude == 0 && msg->altitude == 0){
					ROS_WARN("GPS signal lost!");
					return; 
				}

				latitude = msg->latitude;
        		longitude = msg->longitude;
        		h = msg->altitude;

        		//lla to ecef
        		lamb = deg_to_rad*(latitude);
        		phi = deg_to_rad*(longitude);
        		s = sin(lamb);
        		N = a / sqrt(1 - e_sq * s * s);

        		sin_lambda = sin(lamb);
        		cos_lambda = cos(lamb);
        		sin_phi = sin(phi);
        		cos_phi = cos(phi);

        		x = (h + N) * cos_lambda * cos_phi;
        		y = (h + N) * cos_lambda * sin_phi;
        		z = (h + (1 - e_sq) * N) * sin_lambda;

        		xd = x - x0;
        		yd = y - y0;
        		zd = z - z0;

        		xEast = -sin_phi * xd + cos_phi * yd;
        		yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        		zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

        		ROS_INFO("ENU position: [%f,%f, %f]", xEast, yNorth,zUp);

        		//publish the odometry/tf message over ROS

				transform.setOrigin( tf::Vector3(xEast, yNorth, zUp) );
        		tf::Quaternion q;
        		q.setRPY(0, 0, 0);
				transform.setRotation(q);
				tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", ros::this_node::getName()));

        		odom.header.stamp = ros::Time::now();

				odom.header.frame_id = "map";

        		odom.pose.pose.position.x = xEast;
        		odom.pose.pose.position.y = yNorth;
        		odom.pose.pose.position.z = zUp;

        		odom_pub.publish(odom);
   			}

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "pub_sub");
    pub_sub my_pub_sub;
    ros::spin();
    return 0;
}
