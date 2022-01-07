#include "routing_delta/dij.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "routing");
    
    ros::NodeHandle nh("~");

    dij::Dij dij_planner(nh);

    std::string vertexFilePath, edgeFilePath, pcdFilePath;
    
    // vertexFilePath = "src/perception/simu_data/Vertex.yaml";
    // edgeFilePath = "src/perception/simu_data/Edge.yaml";
    // pcdFilePath = "src/perception/simu_data/";
    
    nh.getParam("vertexFilePath", vertexFilePath);
    nh.getParam("edgeFilePath", edgeFilePath);
    nh.getParam("pcdFilePath", pcdFilePath);

    TopoMap topoMap = loadMap(vertexFilePath, edgeFilePath, pcdFilePath);

    std::cout << "Please input source id" << std::endl;

    std::cin >> dij_planner.source_id;

    std::cout << "Please input target id" << std::endl;
    
    std::cin >> dij_planner.target_id;

    if (dij_planner.source_id == dij_planner.target_id)
    {
        std::cout << "The source is the same as the target" << std::endl;

        exit(0);
    }
    
    if (!dij_planner.FindPath(topoMap))
    {
        std::cout << "Can not find a path" << std::endl;

        exit(0);
    }

    std::cout << "A path has been found" << std::endl;
    
    dij_planner.GenPath();

    ros::Rate loop_rate(dij_planner.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        dij_planner.PubPath();

        dij_planner.PubTarget(topoMap);

        loop_rate.sleep();
    }

    return 0;
}
