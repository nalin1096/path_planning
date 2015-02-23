#ifndef OBSTACLES_H
#define OBSTACLES_H
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>

using namespace std;

class obstacles
{
    public:
        /** Default constructor */
        obstacles() {}
        /** Default destructor */
        virtual ~obstacles() {}

        vector< vector<geometry_msgs::Point> > getObstacleArray();

    protected:
    private:
        vector< vector<geometry_msgs::Point> > obstacleArray;
};

#endif // OBSTACLES_H
