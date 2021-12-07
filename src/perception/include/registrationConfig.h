#pragma once

struct RegistrationConfig
{
    float x_pre;
    float y_pre;
    float yaw_pre;

    float maximumIterations;
    float resolution;
    float stepSize;
    float transformationEpsilon;
    float maxCorrespondenceDistance;
    float euclideanFitnessEpsilon;
    
    double yaw_thre;
    double fitnessScore_thre;
    double minFitnessScore_thre;

    bool menu_bool;
};
