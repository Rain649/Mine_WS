#include "control_underground_delta/path_follow.h"

bool PathFollow::CallbackFlag(void)
{
    if (path_counter > Counter_MAX)
    {
        path_flag = false;
        path_counter = Counter_MAX;
    }
    
    // std::cout << "odom_flag = " << odom_flag << std::endl;
    std::cout << "veh_flag = " << veh_flag << std::endl;
    std::cout << "path_flag = " << path_flag << std::endl;

    return veh_flag && path_flag;

    // return odom_flag && veh_flag && path_flag;
    
    // return odom_flag && path_flag;

    // return odom_flag && veh_flag && path_flag && pid_flag;
}

void PathFollow::PushPathNode(void)
{
    // 找最近路点
    dis_min = 1000; // 初始化足够大即可

    near_num = 0;

    for (size_t i = 0; i < path_ref.size(); i++)
    {
        dis_tmp = sqrt(pow(path_ref[i].x - state_now.x, 2) + 
                       pow(path_ref[i].y - state_now.y, 2));
        
        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;

            near_num = i;
        }
    }

    // 去掉车后经过的点
    for (size_t i = 0; i < near_num; i++)
    {
        path_ref.erase(path_ref.begin());
    }
}

void PathFollow::GenLatCmd(void)
{
    ++path_counter;

    pre_dis = pre_dis_min + u_weight * fabs(state_now.u);

    std::cout << "pre_dis = " << pre_dis << std::endl;

    // 找最近路点
    dis_min = 1000; // 初始化足够大即可

    near_num = 0;

    for (size_t i = 0; i < path_ref.size(); i++)
    {
        dis_tmp = sqrt(pow(path_ref[i].x - state_now.x, 2) + 
                       pow(path_ref[i].y - state_now.y, 2));
        
        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;

            near_num = i;
        }
    }

    std::cout << "The Nearest Point is No." << near_num << std::endl;

    // 找目标路点
    aim_num = 0;

    if (dis_min > pre_dis)
    {
        aim_num = near_num;
    }
    else
    {
        for (size_t i = 0; i < path_ref.size(); i++)
        {
            if (path_ref[i].s - path_ref[near_num].s > pre_dis)
            {
                aim_num = i;

                break;
            }
            
            if (i == path_ref.size()-1)
                aim_num = path_ref.size()-1;
        }
    }

    std::cout << "The Aim Point is No." << aim_num << std::endl;

    // 在接近路径终点处停车
    // if (aim_num == path_ref.size() - 1)
    // {
    //     if (path_ref[aim_num].s - path_ref[near_num].s < Near_dis)
    //     {
    //         std::cout << "Near Target" << std::endl;

    //         Stop();

    //         exit(0);            
    //     }
    // }

    // 单点预瞄
    aim_dis = sqrt(pow(path_ref[aim_num].x - state_now.x, 2) + 
                   pow(path_ref[aim_num].y - state_now.y, 2));

    pre_angle = atan2(path_ref[aim_num].y - state_now.y, path_ref[aim_num].x - state_now.x) - state_now.yaw;

    if (fabs(pre_angle) < 1e-2)
    {
        cmd_steer = 0;

        cmd_sim.angular.z = 0;
    }
    else
    {
        pre_r = aim_dis/2/sin(pre_angle);

        cmd_steer = atan(Veh_L/pre_r);

        // sim cmd
        if (fabs(state_now.u) == 0)
        {
            cmd_sim.angular.z = cmd_sim_lon / pre_r;
        }
        else
        {
            cmd_sim.angular.z = fabs(state_now.u) / pre_r;
        }
    }
    
    if (cmd_steer > Steer_max)
        cmd_steer = Steer_max;
    
    if (cmd_steer < -Steer_max)
        cmd_steer = -Steer_max;
    
    cmd.data[1] = cmd_steer;
}

void PathFollow::GenLonCmd(void)
{
    // sim cmd
    if (fabs(cmd_sim.angular.z) > cmd_sim_lat * 0.5)
    {
        cmd_sim.linear.x = cmd_sim_lon * 0.2;
    }
    else if (fabs(cmd_sim.angular.z) > cmd_sim_lat * 0.3)
    {
        cmd_sim.linear.x = cmd_sim_lon * 0.5;
    }
    else
    {
        cmd_sim.linear.x = cmd_sim_lon;
    }

    return;

    // // 起步    
    // counter++;

    // if (counter < 100)
    // {
    //     // cmd.data[0] = 25;
    //     cmd.data[0] = Torque_startup;

    //     return;
    // }
    // else
    // {
    //     counter = 100;
    // }

    // 根据最近点到预瞄点间的参考路径曲率确定目标车速
    if (near_num == aim_num)
    {
        u_aim = path_ref[aim_num].u;
    }
    else
    {
        u_sum = 0;

        for (size_t i = near_num; i < aim_num; i++)
        {
            u_sum += path_ref[i].u;
        }
        
        u_aim = u_sum/(aim_num - near_num);
    }

    std::cout << "u_aim = " << u_aim << "m/s, " << u_aim*3.6 << "km/h" << std::endl;
    
    std::cout << "state_now.u = " << state_now.u << "m/s, " << state_now.u*3.6 << "km/h" << std::endl;

    error_u = u_aim - state_now.u;

    std::cout << "error_u = " << error_u << std::endl;

    if (cmd_torque_old < Torque_max && cmd_torque_old > Torque_min)
    {
        error_deque.push_back(error_u);
    }
    else
    {
        if (cmd_torque_old == Torque_max && error_u < 0)
        {
            error_deque.push_back(error_u);
        }
        else
        {
            if (cmd_torque_old == Torque_min && error_u > 0)
            {
                error_deque.push_back(error_u);
            }
        }
    }
    
    if (error_deque.size() > Error_size_max)
        error_deque.pop_front();
    
    error_sum = 0;

    for (size_t i = 0; i < error_deque.size(); i++)
    {
        error_sum += error_deque[i];
    }
    
    std::cout << "error_sum = " << error_sum << std::endl;

    std::cout << "pid_p = " << pid_p << std::endl;
    std::cout << "pid_i = " << pid_i << std::endl;
    std::cout << "pid_d = " << pid_d << std::endl;
    
    cmd_torque = pid_p * error_u + 
                 pid_i * error_sum / RATE + 
                 pid_d * (error_u - error_u_old) * RATE;
    
    if (cmd_torque > cmd_torque_old + Torque_delta)
        cmd_torque = cmd_torque_old + Torque_delta;
    
    if (cmd_torque < cmd_torque_old - Torque_delta)
        cmd_torque = cmd_torque_old - Torque_delta;
    
    // if (cmd_torque < 0 && cmd_torque_old > 0)
    //     cmd_torque = 0;
    
    // if (cmd_torque > 0 && cmd_torque_old < 0)
    //     cmd_torque = 0;
    
    if (cmd_torque > Torque_max)
        cmd_torque = Torque_max;
    
    if (cmd_torque < Torque_min)
        cmd_torque = Torque_min;
    
    error_u_old = error_u;

    cmd_torque_old = cmd_torque;

    cmd.data[0] = cmd_torque;

    // cmd.data[0] = u_aim;
}

void PathFollow::Stop(void)
{
    cmd.data[0] = 0;
    cmd.data[1] = 0;

    // CmdPub.publish(cmd);

    cmd_sim.linear.x = 0;
    cmd_sim.angular.z = 0;

    std::cout << "stop" << std::endl;
}