#include "control_underground_delta/path_follow.h"

void PathFollow::PubCmd(void)
{
    // std::cout << "cmd_torque = " << cmd.data[0] << std::endl;
    // std::cout << "cmd_steer = " << cmd.data[1] << std::endl;

    // CmdPub.publish(cmd);

    std::cout << "cmd_v = " << cmd_sim.linear.x << std::endl;
    std::cout << "cmd_w = " << cmd_sim.angular.z << std::endl;

    CmdSimPub.publish(cmd_sim);
}