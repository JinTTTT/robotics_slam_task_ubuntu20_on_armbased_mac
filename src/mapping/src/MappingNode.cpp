#include "mapping/Mapping.h"
#include "mapping/Utils.h"

using namespace std;




MappingNode::MappingNode(ros::NodeHandle n) : // definition of the constructor
	nodeHandle(n),
	laserScanSubscriber(nodeHandle, "scan", 5),
	laserScanFilter(laserScanSubscriber, tfListener, "odom", 5)  // initialization list
{
	// initialize map meta data
	// map is 20 x 20 meters, resolution is 0.02 meters per grid cell
	 
	this->map.info.width = 1000; // 1000 cells * 0.02 m/cell = 20 m
	this->map.info.height = 1000;
	// origin position is bottom left corner of the map : (-10 m, -10 m)
	this->map.info.origin.position.x = -this->map.info.resolution * this->map.info.width / 2.0; // - 0.02 * 1000 / 2 = -10 m
	this->map.info.origin.position.y = -this->map.info.resolution * this->map.info.height / 2.0; // - 0.02 * 1000 / 2 = -10 m
	this->map.info.origin.position.z = 0.0;
	// no rotation of the map
	this->map.info.origin.orientation.x = 0.0;
	this->map.info.origin.orientation.y = 0.0;
	this->map.info.origin.orientation.z = 0.0;
	this->map.info.origin.orientation.w = 1.0;

	this->map.data.resize( this->map.info.width * this->map.info.height );

	// initialize map, set all cells to -1 (unknown)
	// unsigned means non negative integer, double the range of the index
	// prevents accidental negative index
	for (unsigned int i = 0; i < this->map.info.width; i++) {
		for (unsigned int j = 0; j < this->map.info.height; j++) {
			int index = j*this->map.info.height + i;
			this->map.data[index] = -1; // -1 means unknown
		}
	}
	///////////////////////////////////////////////////
	// <your code>

	// YOU may initialize all the data structures you need
	// to implement simple counting method here!

	// initialize each value of the hit_miss_counter as (0,0)
	hit_miss_counter.resize(this->map.info.width * this->map.info.height); // to 1D vector
	for (unsigned int i = 0; i < this->map.info.width; i++) {
		for (unsigned int j = 0; j < this->map.info.height; j++) {
			int index = j*this->map.info.height + i;
			hit_miss_counter[index] = pair<int, int>(0, 0); // (0 hits, 0 passes)
		}
	}
	
	// </your code>
	///////////////////////////////////////////////////

	// subscribe to laser scan
	// when date _1 received, call this(MappingNode) use &MappingNode::laserReceived
	laserScanFilter.registerCallback( boost::bind( &MappingNode::laserReceived, this, _1 ) );
//	laserScanFilter.setTolerance(ros::Duration(0.01));

	// publish the map we're building
	// advertice<nav_msgs::OccupancyGrid> is a template function, with the type info, it can automatically determine the type of the message to publish
	this->mapPublisher = this->nodeHandle.advertise<nav_msgs::OccupancyGrid>( "map", 2 );
	// to pass a function to another function, we always have to pass the reference of the object, you cannot copy the function like a variable
	this->mapService = this->nodeHandle.advertiseService( "map", &MappingNode::mapRequested, this );

	// publish the unknown map for the first time
	this->mapPublisher.publish( this->map );
}

MappingNode::~MappingNode() {
	// no need to do anything here, ros nodehandle will be destroyed automatically
}


void MappingNode::laserReceived( const sensor_msgs::LaserScanConstPtr& laserScan ) {
	// what's the robot's odometry when this scan was taken?

	// use timestamp from laser scan header
	// ident is a varible name, like std::string ident = "laser"; but can also be written as std::string ident( "laser" );
	tf::Stamped<tf::Pose> ident( tf::Transform( tf::createIdentityQuaternion(), tf::Vector3(0,0,0) ), laserScan->header.stamp, "laser" );
	// we want to know the /odom at this timestamp, because after laserscan triggered,  there can be some delay that we get the odom data
	tf::Stamped<tf::Pose> odomPose;
	try {
		this->tfListener.transformPose( "odom", ident, odomPose ); // transform from laser frame to odom frame
	}
	catch( tf::TransformException e ) {
		ROS_WARN( "Failed to compute odom pose, skipping scan (%s)", e.what() );
		return;
	}

    // transform pose of robot to map coordinates. funit from [m] to [cells]
    int robotX = odomPose.getOrigin().x() / this->map.info.resolution + this->map.info.width / 2;
    int robotY = odomPose.getOrigin().y() / this->map.info.resolution + this->map.info.height / 2;
    double robotTheta = tf::getYaw( odomPose.getRotation() );

    ///////////////////////////////////////////////////
	// <your code>

	// compute the reflection grid update value here
	//  you should use updateReflectionMapValue(int x, int y, double value) for assigning the value
	//  that you computed
	
	// 	the method bresenhamLine( int x0, int y0, int x1, int y1 ) will also be helpful.
	
	// odomPose contains the *current* pose of the robot, estimated from past odometry data.
    // You can use robotX, robotY, and robotTheta for the robot position and rotation on the map.
	
	// for each laser beam in the scan
	for ( int i = 0; i < laserScan->ranges.size(); i++) {
		// length of the beam
		double range = laserScan->ranges[i];

		// compute the angle of this beam in the world frame
		double beamAngle = laserScan->angle_min + i * laserScan->angle_increment;
		double worldAngle = robotTheta + beamAngle;

		// compute the end point of the beam in world frame
		double endX = odomPose.getOrigin().x() + range * cos( worldAngle );
		double endY = odomPose.getOrigin().y() + range * sin( worldAngle );

		int endCellX = endX / this->map.info.resolution + this->map.info.width / 2;
		int endCellY = endY / this->map.info.resolution + this->map.info.height / 2;

		// check the range if whitin the laser limits
		if ( endCellX < 0 || endCellY < 0 || endCellX >= this->map.info.width || endCellY >= this->map.info.height ) {
			// out of map
			continue; // skip this beam
		}

		// get all cells that are passed by this beam: output like (x0, y0), (x1, y1), ..., (xn, yn)
		vector< pair<int, int> > cells = bresenhamLine( robotX, robotY, endCellX, endCellY );
		
		// process all cells except the last one ( the hit cell )
		for ( int j = 0; j < cells.size() - 1; j++) {
			int cellX = cells[j].first;
			int cellY = cells[j].second;
			int index = cellX + cellY * this->map.info.height; // convert to 1D index

			// update miss count
			hit_miss_counter[index].second += 1; // increment passes

			// calculate the reflection value and update
			int hits = hit_miss_counter[index].first;
			int passes = hit_miss_counter[index].second;
			double probability = (double)hits / (double)(hits + passes);
			updateReflectionMapValue(cellX, cellY, probability);
		}

		// process the hit cell
		int hitCellX = cells.back().first;
		int hitCellY = cells.back().second;
		int hitIndex = hitCellX + hitCellY * this->map.info.height;

		// update hit count
		hit_miss_counter[hitIndex].first += 1; // increment hits

		// calculate the reflection value and update
		int hits = hit_miss_counter[hitIndex].first;
		int passes = hit_miss_counter[hitIndex].second;
		double probability = (double)hits / (double)(hits + passes);
		updateReflectionMapValue(hitCellX, hitCellY, probability);

	}



	// </your code>
	///////////////////////////////////////////////////

	// publish result
	this->map.header.stamp = ros::Time::now();
	this->map.header.frame_id = "/odom";
	this->mapPublisher.publish( this->map );
}

void MappingNode::updateReflectionMapValue(int i, int j, double value) {
	int index = j*this->map.info.width + i;
	this->map.data[index] = 100*value; // *100 is just for better visibility in visualization
}

bool MappingNode::mapRequested( nav_msgs::GetMap::Request &request, nav_msgs::GetMap::Response &response ) {
	if( this->map.info.width && this->map.info.height )	{
		this->map.header.stamp = ros::Time::now();
		response.map = this->map;
		return true;
	}
	else
		return false;
}


int main( int argc, char** argv ) {
	ros::init( argc, argv, "mapper" );
	ros::NodeHandle nh;

	MappingNode mn( nh );

	ros::spin();

	return 0;
}
