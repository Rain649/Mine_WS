#include "dubins_bug_delta/dubins_bug.h"
#include "r_s_planner/dubins.h"

bool DBPlanner::CallbackFlag(void)
{
    if (odom_counter > Counter_MAX)
    {
        odom_flag = false;

        odom_counter = Counter_MAX;
    }

    if (target_counter > Counter_MAX)
    {
        target_flag = false;

        target_counter = Counter_MAX;
    }
    
    if (lidar_counter > Counter_MAX)
    {
        lidar_flag = false;

        lidar_counter = Counter_MAX;
    }
    
    ROS_INFO("odom_flag = %d, target_flag = %d, lidar_flag = %d", odom_flag, target_flag, lidar_flag);

    return odom_flag && target_flag && lidar_flag;
}

void DBPlanner::BugPlanner(DubinsStateSpace& db)
{
    std::cout << "cur_x = " << current_state.x << ' '
              << "cur_y = " << current_state.y << ' '
              << "cur_yaw = " << current_state.yaw << std::endl;

    std::cout << "tar_x = " << final_state[0] << ' '
              << "tar_y = " << final_state[1] << ' '
              << "tar_yaw = " << final_state[2] << std::endl;

    dis_v2t_now = sqrt(pow(current_state.x - final_state[0], 2) + 
                       pow(current_state.y - final_state[1], 2));
    
    if (db_rs_flag)
    {
        std::cout << "Distance to target in dubins path = " << db_rs_length << std::endl;

        if (db_rs_length < Tar_tolerance)
        {
            std::cout << "Near Target" << std::endl;

            path_local.clear();
            path_local_veh.clear();
            
            return;
        }
    }
    else
    {
        std::cout << "Distance to target = " << dis_v2t_now << std::endl;
    }

    target_x_veh = (final_state[0] - current_state.x)*cos(current_state.yaw) + 
                   (final_state[1] - current_state.y)*sin(current_state.yaw);
    
    target_y_veh = -(final_state[0] - current_state.x)*sin(current_state.yaw) + 
                    (final_state[1] - current_state.y)*cos(current_state.yaw);

    for (size_t i = 0; i < laser_edge_point.size(); i++)
    {
        laser_edge_point[i].dis_tar = sqrt(pow(laser_edge_point[i].x_veh - target_x_veh, 2) + 
                                           pow(laser_edge_point[i].y_veh - target_y_veh, 2));
    }

    MotionToGoal(db);

    if (dis_v2t_now < Laser_Edge && db_rs_flag)
    {
        dis_v2t_old = dis_v2t_now;
    }
    else
    {
        if (dis_v2t_now < dis_v2t_old)
            dis_v2t_old = dis_v2t_now;
    }

    ++odom_counter;
    ++target_counter;
    ++lidar_counter;
}

void DBPlanner::MotionToGoal(DubinsStateSpace& db)
{
    std::cout << "Motion to Goal" << std::endl;

    path_local.clear();
    path_local_veh.clear();

    // 目标位于雷达边界之内
    if (dis_v2t_now < Laser_Edge)
    {
        // 按期望航向生成 dubins path，并进行碰撞检测
        db_rs_path.clear();

        db.sample(start_state, final_state, step_size, db_rs_length, db_rs_path);

        path_tmp.clear();

        for (auto &point_itr : db_rs_path)
        {
            node_tmp.x = point_itr[0];
            node_tmp.y = point_itr[1];
            node_tmp.phi = point_itr[2];

            path_tmp.push_back(node_tmp);
        }

        db_rs_flag = false;

        PathWorld2Veh(path_tmp, path_tmp_veh);

        obs_index = ObsCheck(path_tmp_veh, 0);

        if (obs_index == -1)
        {
            std::cout << "Approaching target with required yaw" << std::endl;

            db_rs_flag = true;
        }
        else
        {
            // 按目标相对位置生成 dubins path，并进行碰撞检测
            final_state_veh[0] = target_x_veh;
            final_state_veh[1] = target_y_veh;
            final_state_veh[2] = atan2(final_state_veh[1], final_state_veh[0]);

            db_rs_path.clear();

            db.sample(start_state_veh, final_state_veh, step_size, db_rs_length, db_rs_path);

            path_tmp_veh.clear();

            for (auto &point_itr : db_rs_path)
            {
                node_tmp.x = point_itr[0];
                node_tmp.y = point_itr[1];
                node_tmp.phi = point_itr[2];

                path_tmp_veh.push_back(node_tmp);
            }

            obs_index = ObsCheck(path_tmp_veh, 0);
            
            if (obs_index == -1)
            {
                std::cout << "Approaching target without required yaw" << std::endl;
                
                db_rs_flag = true;

                PathVeh2World(path_tmp, path_tmp_veh);
            }
        }
        
        // 若 dubins path 通过碰撞检测，则生成 path local
        if (db_rs_flag)
        {
            path_local = path_tmp;

            path_local_veh = path_tmp_veh;

            GenPathPara(path_local);
            
            return;
        }
    }

    // 目标位于雷达边界之外，或目标在边界内但没有无碰路径，选离目标最近的无障碍点，并生成对应的 dubins path
    while (!free_point_dis.empty())
    {
        free_point_dis.pop();
    }
    
    for (size_t i = 0; i < laser_edge_point.size(); i++)
    {
        free_dis_tmp.x_veh = laser_edge_point[i].x_veh;
        free_dis_tmp.y_veh = laser_edge_point[i].y_veh;

        free_dis_tmp.yaw_veh = laser_edge_point[i].yaw_veh;

        free_dis_tmp.dis_veh = laser_edge_point[i].dis_veh;
        free_dis_tmp.dis_tar = laser_edge_point[i].dis_tar;

        free_dis_tmp.dis_cost = free_dis_tmp.dis_veh + free_dis_tmp.dis_tar;

        free_point_dis.push(free_dis_tmp);
    }
    
    while (!free_point_dis.empty())
    {
        final_state_veh[0] = free_point_dis.top().x_veh;
        final_state_veh[1] = free_point_dis.top().y_veh;
        final_state_veh[2] = free_point_dis.top().yaw_veh;

        db_rs_path.clear();

        db.sample(start_state_veh, final_state_veh, step_size, db_rs_length, db_rs_path);

        path_tmp_veh.clear();

        for (auto &point_itr : db_rs_path)
        {
            node_tmp.x = point_itr[0];
            node_tmp.y = point_itr[1];
            node_tmp.phi = point_itr[2];

            path_tmp_veh.push_back(node_tmp);
        }

        obs_index = ObsCheck(path_tmp_veh, 0);
        
        if (obs_index == -1)
            break;
        
        free_point_dis.pop();
    }
    
    if (free_point_dis.empty())
    {
        std::cout << "free_point_dis is empty" << std::endl;
    }
    else
    {
        path_local_veh = path_tmp_veh;

        PathVeh2World(path_local, path_local_veh);

        GenPathPara(path_local);
    }
}

bool DBPlanner::UpdatePdtPath(void)
{
    // 若无预测轨迹，更新
    if (path_pdt.empty())
    {
        std::cout << "path_pdt is empty" << std::endl;

        return true;
    }
    else
    {
        PathWorld2Veh(path_pdt, path_pdt_veh);

        near_index = FindNearPoint(path_pdt, current_state);

        s_threshold = path_pdt[floor(path_pdt.size()/S_proportion)].s;

        // 若到达预测轨迹距离阈值，更新
        if (path_pdt[near_index].s > s_threshold)
        {
            // std::cout << "path_pdt needs to be updated because of length" << std::endl;

            return true;
        }

        // 若预测轨迹有障碍，更新
        obs_index = ObsCheck(path_pdt_veh, near_index);

        if (obs_index != -1)
        {
            // std::cout << "path_pdt needs to be updated because of obstacle" << std::endl;

            return true;
        }

        // 如果规划路径的航向变化过大，更新
        if (!path_local.empty())
        {
            if (fabs(path_pdt.back().phi - path_local.back().phi) > Phi_alter)
            {
                // std::cout << "path_pdt needs to be updated because of yaw" << std::endl;
                
                return true;
            }
        }
    }
    
    return false;
}

void DBPlanner::GenPdtPath(void)
{
    path_pdt = path_local;

    path_pdt_veh = path_local_veh;
}

int DBPlanner::ObsCheck(std::vector<PathNode> &path, int near_index)
{
    if (path.size() < 2)
        return 0;

    for (size_t i = near_index; i < path.size(); i++)
    {
        // 三碰撞圆检测
        index_row = map_rows-1 - floor((path[i].y + Lf*sin(path[i].phi) - map_o_y)/map_res);
        index_col = floor((path[i].x + Lf*cos(path[i].phi) - map_o_x)/map_res);
            
        if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
        {
            if (image_dis.at<float>(index_row, index_col)*map_res < OBS_DIS)
                return i;
        }

        index_row = map_rows-1 - floor((path[i].y - map_o_y)/map_res);
        index_col = floor((path[i].x - map_o_x)/map_res);
            
        if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
        {
            if (image_dis.at<float>(index_row, index_col)*map_res < OBS_DIS)
                return i;
        }

        index_row = map_rows-1 - floor((path[i].y - Lr*sin(path[i].phi) - map_o_y)/map_res);
        index_col = floor((path[i].x - Lr*cos(path[i].phi) - map_o_x)/map_res);

        if (index_row >= 0 && index_row < map_rows && index_col >= 0 && index_col < map_cols)
        {
            if (image_dis.at<float>(index_row, index_col)*map_res < OBS_DIS)
                return i;
        }
    }

    return -1;
}

void DBPlanner::PathWorld2Veh(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v)
{
    path_v.resize(path_w.size());

    for (size_t i = 0; i < path_v.size(); i++)
    {
        path_v[i].x = (path_w[i].x - current_state.x)*cos(current_state.yaw) +
                      (path_w[i].y - current_state.y)*sin(current_state.yaw);
        
        path_v[i].y = -(path_w[i].x - current_state.x)*sin(current_state.yaw) +
                       (path_w[i].y - current_state.y)*cos(current_state.yaw);
    }

    if (path_v.size() < 2)
        return;
    
    for (size_t i = 0; i < path_v.size(); i++)
    {
        if (i == 0)
        {
            path_v[i].phi = atan2(path_v[i+1].y - path_v[i].y, path_v[i+1].x - path_v[i].x);
        }
        else
        {
            if (i == path_v.size()-1)
                path_v[i].phi = atan2(path_v[i].y - path_v[i-1].y, path_v[i].x - path_v[i-1].x);
            else
                path_v[i].phi = atan2(path_v[i+1].y - path_v[i-1].y, path_v[i+1].x - path_v[i-1].x);                
        }
    }
}

void DBPlanner::PathVeh2World(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v)
{
    path_w.resize(path_v.size());

    for (size_t i = 0; i < path_v.size(); i++)
    {
        path_w[i].x = path_v[i].x * cos(-current_state.yaw) + 
                      path_v[i].y * sin(-current_state.yaw) + current_state.x;
        
        path_w[i].y = -path_v[i].x * sin(-current_state.yaw) + 
                       path_v[i].y * cos(-current_state.yaw) + current_state.y;
    }

    if (path_w.size() < 2)
        return;

    for (size_t i = 0; i < path_w.size(); i++)
    {
        if (i == 0)
        {
            path_w[i].phi = atan2(path_w[i+1].y - path_w[i].y, path_w[i+1].x - path_w[i].x);            
        }
        else
        {
            if (i == path_w.size()-1)
                path_w[i].phi = atan2(path_w[i].y - path_w[i-1].y, path_w[i].x - path_w[i-1].x);
            else
                path_w[i].phi = atan2(path_w[i+1].y - path_w[i-1].y, path_w[i+1].x - path_w[i-1].x);
        }
    }
}

int DBPlanner::FindNearPoint(std::vector<PathNode> &path, VehState &veh)
{
    near_index = 0;

    dis_min = Infinity;

    for (size_t i = 0; i < path.size(); i++)
    {
        dis_tmp = sqrt(pow(path[i].x - veh.x, 2) + pow(path[i].y - veh.y, 2));

        if (dis_tmp < dis_min)
        {
            dis_min = dis_tmp;
            near_index = i;
        }
    }

    return near_index;
}

void DBPlanner::GenPathPara(std::vector<PathNode> &path)
{
    // 生成沿路径到起点的长度
    for (size_t i = 0; i < path.size(); i++)
    {
        if (i == 0)
        {
            path[i].s = 0;
        }
        else
        {
            path[i].s = path[i-1].s + sqrt(pow(path[i].y - path[i-1].y, 2) +
                                           pow(path[i].x - path[i-1].x, 2));
        }
    }
}