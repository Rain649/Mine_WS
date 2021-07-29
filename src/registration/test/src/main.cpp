#ifdef _OPENMP
#include <omp.h>
#endif

// Standard
#include <iostream>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>

// PRT
#include "types.hpp"
#include "Registrator.hpp"
#include "Visualizer.hpp"
#include "util.hpp"

using namespace PRT;

int main(int argc, char *argv[])
{
    int num_ksearch_neighbors = 100;
    double descriptor_radius = 1.0;
    double subsampling_radius = 0.25;
    double consensus_inlier_threshold = 0.5;
    int consensus_max_iterations = 100;
    int icp_max_iterations = 100;
    double icp_max_correspondence_distance = 0.05;
    double residual_threshold = 0.25;
    std::string registration_technique = "both";

    std::string usage_message = "\nUsage:\n\n1. ./point_cloud_registration_tool [options] <source_point_cloud> <target_point_cloud>\n\nSee --help for optional arguments\n";

    if (descriptor_radius <= 0)
    {
        std::cout << "Error: Descriptor radius must be a positive value." << std::endl;
        return -1;
    }

    if (subsampling_radius <= 0)
    {
        std::cout << "Error: Subsampling radius must be a positive value." << std::endl;
        return -1;
    }

    if (consensus_inlier_threshold <= 0)
    {
        std::cout << "Error: Consensus inlier threshold must be a positive value." << std::endl;
        return -1;
    }

    if (icp_max_correspondence_distance <= 0)
    {
        std::cout << "Error: Maximum correspondence distance must be a positive value." << std::endl;
        return -1;
    }

    if (residual_threshold <= 0)
    {
        std::cout << "Error: Residual threshold must be a positive value." << std::endl;
        return -1;
    }

    if (argc < 3)
    {
        std::cout << "Input enough parameters" << std::endl;
        return 0;
    }

    //Hide the Debug Informatica
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    FilepairVectorPtr filepairs;
    filepair_t filePair;
    filePair.sourcefile = argv[1];
    filePair.targetfile = argv[2];

#ifdef _OPENMP
    omp_set_nested(true);
#endif

    PointCloudT target_cloud;
    PointCloudT source_cloud;
    std::string target_cloud_filepath = filePair.targetfile;
    std::string source_cloud_filepath = filePair.sourcefile;
    bool reads_successful = true;

    if (util::loadPointCloud(source_cloud_filepath, source_cloud) != 0)
    {
        std::cout << "Invalid input:" << std::endl
                  << source_cloud_filepath << std::endl;
        std::cout << "Skipping.." << std::endl;
        reads_successful = false;
    }

    if (util::loadPointCloud(target_cloud_filepath, target_cloud) != 0)
    {
        std::cout << "Invalid input:" << std::endl
                  << target_cloud_filepath << std::endl;
        std::cout << "Skipping.." << std::endl;
        reads_successful = false;
    }

    if (!reads_successful)
        return -1;

    //Extract prefix for output files
    std::string prefix = util::removeFileExtension(filePair.sourcefile);

    //Set output file paths
    std::string transformation_matrix_filepath;
    std::string registered_pointcloud_filepath;
    std::string residual_histogram_image_filepath;
    std::string fscore_filepath;

    transformation_matrix_filepath = prefix + "_transform.csv";

    registered_pointcloud_filepath = prefix + "_registered.pcd";

    residual_histogram_image_filepath = prefix + "_residual_histogram.png";

    fscore_filepath = prefix + "_fscore.txt";

    //Ensure unique output filepaths
    if (util::ensureUniqueFilepath(transformation_matrix_filepath) != 0)
    {
        std::cout << "Failed to create transformation matrix file." << std::endl;
        return -1;
    }

    if (util::ensureUniqueFilepath(fscore_filepath) != 0)
    {
        std::cout << "Failed to create f-score file." << std::endl;
        return -1;
    }

    if (util::ensureUniqueFilepath(registered_pointcloud_filepath) != 0)
    {
        std::cout << "Failed to create registered PCD file." << std::endl;
        return -1;
    }

    if (util::ensureUniqueFilepath(residual_histogram_image_filepath) != 0)
    {
        std::cout << usage_message << std::endl;
        return -1;
    }

    //Registration
    //Setup
    Registrator::Ptr registrator(new Registrator());
    registrator->setNumKSearchNeighbors(num_ksearch_neighbors);
    registrator->setDescriptorRadius(descriptor_radius);
    registrator->setSubsamplingRadius(subsampling_radius);
    registrator->setConsensusInlierThreshold(consensus_inlier_threshold);
    registrator->setConsensusMaxIterations(consensus_max_iterations);
    registrator->setICPMaxIterations(icp_max_iterations);
    registrator->setICPMaxCorrespondenceDistance(icp_max_correspondence_distance);
    registrator->setResidualThreshold(residual_threshold);
    registrator->setTargetCloud(target_cloud.makeShared());
    registrator->setSourceCloud(source_cloud.makeShared());

    //Compute
    registrator->performRegistration(registration_technique);

    //Save Results
    registrator->saveResidualColormapPointCloud(registered_pointcloud_filepath);
    // registrator->saveFinalTransform(transformation_matrix_filepath);
    // registrator->saveFScoreAtThreshold(fscore_filepath, residual_threshold);

    std::cout << "Registration of " << source_cloud_filepath << " finished" << std::endl;
    std::cout << "F-score: " << registrator->getFScoreAtThreshold() << std::endl;
    std::cout << "Saved to: " << registered_pointcloud_filepath << std::endl;

    //Visualization
    Visualizer visualizer("Point Cloud Registration");
    visualizer.setRegistrator(registrator);
    // visualizer.saveHistogramImage(residual_histogram_image_filepath);
    visualizer.visualize();
    registrator.reset();

    return 0;
}
