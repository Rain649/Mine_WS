/*此程序由北京理工大学*刘仕杰*编写*/
#include <iostream>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <iostream>
// #include <vector>

using namespace std;

struct Position
{
   float x, y, z;
};

struct Intersection
{
    int index;
    Position position;
    int cluster_number;
    int peak_number;
    int search_radius;
};

void operator >> (const YAML::Node& node, Position& position)
{
   position.x = node[0].as<float>();
   position.y = node[1].as<float>();
   position.z = node[2].as<float>();
}

void operator >> (const YAML::Node& node, Intersection& intersection)
{
   intersection.index = node["index"].as<int>();
   node["position"] >> intersection.position;
   intersection.cluster_number = node["cluster_number"].as<int>();
   intersection.peak_number = node["peak_number"].as<int>();
   intersection.search_radius = node["search_radius"].as<int>();
}

void print(Intersection& intersection)
{
    cout << "Intersection Index: " << intersection.index << endl;
    cout << "Intersection position: (" << intersection.position.x << ", " <<
        intersection.position.y << ", " << intersection.position.z << ")" << endl;
    cout << "Intersection cluster_number: " << intersection.cluster_number << endl;
    cout << "Intersection peak_number: " << intersection.peak_number << endl;
    cout << "Intersection search_radius: " << intersection.search_radius << endl;
    cout << "------------------" << endl;
}

int main(int argc, char **argv)
{
    if(1)
    {
        std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/mapData.yaml";
        YAML::Node map = YAML::LoadFile(fin);
        for(YAML::const_iterator it=map.begin();it!=map.end();++it)
        {
            Intersection j;
            *it >> j;
            print(j);
        }
        // std::ofstream file;
        // file.open(fin);
        // file.flush();
        // file << yamlConfig;
        // file.close();
    }
    
    if(0)
    {
        std::ofstream fout("/home/lsj/dev/Mine_WS/src/preception/include/mapData.yaml");
        YAML::Emitter out(fout);
        out << YAML::BeginMap;
        out << YAML::Key << "int_param";
        out << YAML::Value << 1;
        out << YAML::Key << "double_param";
        out << YAML::Value << 0.5;
        out << YAML::Key << "bool_param";
        out << YAML::Value << false;
        out << YAML::Comment("bool parameter");
        out << YAML::Key << "str_param";
        out << YAML::Value << "test";
        out << YAML::EndMap;
    }

    return 0;
}


    // if(1)
    // {
    //     std::string fin = "/home/lsj/dev/Mine_WS/src/preception/include/mapData.yaml";
    //     YAML::Node yamlConfig = YAML::LoadFile(fin);
    //     int int_param = yamlConfig["int_param"].as<int>();
    //     std::cout << "  node size: " << yamlConfig.size() << std::endl;
    //     std::cout << yamlConfig["bool_param"].as<bool>() << "\n";
    //     yamlConfig["bool_param"] = !yamlConfig["bool_param"].as<bool>();
    //     yamlConfig["double_param"] = yamlConfig["double_param"].as<double>() + 1.0;
    //     yamlConfig["str_param"] = yamlConfig["str_param"].as<string>() + "s";
    //     std::ofstream file;
    //     file.open(fin);
    //     file.flush();
    //     file << yamlConfig;
    //     file.close();
    // }