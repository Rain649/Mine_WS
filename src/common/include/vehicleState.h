#pragma once

struct VehicleState
{
    float x;
    float y;
    float yaw;
    float v;
    float acceleration;
    float delta;

    VehicleState() {}
    VehicleState(float _x, float _y, float _yaw, float _v, float _acceleration, float _delta) : x(_x), y(_y), yaw(_yaw), v(_v), acceleration(_acceleration), delta(_delta) {}
};
