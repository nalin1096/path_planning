#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>

using namespace rrt;

void initializeMarkers(visualization_msgs::Marker &boundary,
    visualization_msgs::Marker &obstacle)
{
    //init headers
	boundary.header.frame_id    = obstacle.header.frame_id    = "path_planner";
	boundary.header.stamp       = obstacle.header.stamp       = ros::Time::now();
	boundary.ns                 = obstacle.ns                 = "path_planner";
	boundary.action             = obstacle.action             = visualization_msgs::Marker::ADD;
	boundary.pose.orientation.w = obstacle.pose.orientation.w = 1.0;

    //setting id for each marker
    boundary.id    = 110;
	obstacle.id   = 111;

	//defining types
	boundary.type  = visualization_msgs::Marker::LINE_STRIP;
	obstacle.type = visualization_msgs::Marker::LINE_LIST;

	//setting scale
	boundary.scale.x = 1;
	obstacle.scale.x = 0.2;

    //assigning colors
	boundary.color.r = obstacle.color.r = 0.0f;
	boundary.color.g = obstacle.color.g = 0.0f;
	boundary.color.b = obstacle.color.b = 0.0f;

	boundary.color.a = obstacle.color.a = 1.0f;
}

vector<geometry_msgs::Point> initializeBoundary()
{
    vector<geometry_msgs::Point> bondArray;

    geometry_msgs::Point point;

    //first point
    point.x = 0;
    point.y = 0;
    point.z = 0;

    bondArray.push_back(point);

    //second point
    point.x = 0;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    //third point
    point.x = 100;
    point.y = 100;
    point.z = 0;

    bondArray.push_back(point);

    //fourth point
    point.x = 100;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    //first point again to complete the box
    point.x = 0;
    point.y = 0;
    point.z = 0;
    bondArray.push_back(point);

    return bondArray;
}

vector<geometry_msgs::Point> initializeObstacles()
{
    vector< vector<geometry_msgs::Point> > obstArray;

    vector<geometry_msgs::Point> obstaclesMarker;

    obstacles obst;

    obstArray = obst.getObstacleArray();

    for(int i=0; i<obstArray.size(); i++)
    {
        for(int j=1; j<5; j++)
        {
            obstaclesMarker.push_back(obstArray[i][j-1]);
            obstaclesMarker.push_back(obstArray[i][j]);
        }

    }
    return obstaclesMarker;
}

int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"env_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher env_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	//defining markers
    visualization_msgs::Marker boundary;
    visualization_msgs::Marker obstacle;

    initializeMarkers(boundary, obstacle);

    //initializing rrtTree
    RRT myRRT(2.0,2.0);
    int goalX, goalY;
    goalX = goalY = 95;

    boundary.points = initializeBoundary();
    obstacle.points = initializeObstacles();

    env_publisher.publish(boundary);
    env_publisher.publish(obstacle);

    while(ros::ok())
    {
        env_publisher.publish(boundary);
        env_publisher.publish(obstacle);
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
	return 1;
}
