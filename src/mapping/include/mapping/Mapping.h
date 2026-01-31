#ifndef MAPPING_MAPPING_H
#define MAPPING_MAPPING_H

#include "ros/ros.h"

// Messages and services that we need
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

using namespace std;

class MappingNode {

public:
	MappingNode(ros::NodeHandle n); // declaration of the constructor, usually put it in header file
	~MappingNode();
// members inside the class can access each other directly
// private members can only be accessed by the members inside the class
// public members can be accessed by the members outside the class
private:
	ros::NodeHandle nodeHandle;

	// we use the OccupancyGrid structure representing the reflection map
	nav_msgs::OccupancyGrid map;

	// subscribe to laser, use a msg_filter to ensure that we can associate an odometry pose with the scan
	tf::TransformListener tfListener;
	message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> laserScanFilter;

	// in order to visualize the map, we have to publish it
	ros::Publisher mapPublisher;
	ros::ServiceServer mapService;

	// Message and service callbacks
	// & laser_scan pass by reference, laser_scan inside is now an alias for the actual laser scan message
	// const means cannot change laser_scan
	void laserReceived( const sensor_msgs::LaserScanConstPtr& laser_scan );
	bool mapRequested( nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res );

	///////////////////////////////////////////////////
	// <helpful methods>

	// implementation of the bresenham algorithm: http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
	//  returns all points from the starting point (x0, y0) and end point (x1, y1)
	//  starting and end point are included in the returned result.
	
	// use this method to update the reflection map
	void updateReflectionMapValue(int x, int y, double value);

	// </helpful methods>
	///////////////////////////////////////////////////
	
	///////////////////////////////////////////////////
	// <your declarations>
	vector<pair<int, int>> hit_miss_counter;  // pair<hits, passes> for each cell

	// </your declarations>
	///////////////////////////////////////////////////
};

#endif