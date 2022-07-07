#ifndef STATE_H
#define STATE_H

#include<vector>

struct State
{
    int vertex_id;
    int branch_num;
    int intersectionID_id;

    double expectedSpeed;
    double steeringAngle;
    double realSpeed;
    double speed;
    double distance;

    bool intersectionVerified;
    bool isLocation;

    std::vector<int> pathPlanned;
};



#endif // STATE_H
