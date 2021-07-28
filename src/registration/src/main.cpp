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

// GFLAGS
// #include <gflags/gflags.h>

// DECLARE_bool(help);
// DEFINE_bool(h, false, "Show help");
// DEFINE_bool(gui, false, "launch GUI after registration");
// DEFINE_string(batch_file, "", "path to batch processing file");
// DEFINE_string(transformation_matrix_filepath, "[source_filename]_transform.csv", "filepath to output transformation matrix CSV");
// // DEFINE_string(registered_pointcloud_filepath, "[source_filename]_registered.ply", "filepath to output registered point cloud PLY");
// DEFINE_string(registered_pointcloud_filepath, "[source_filename]_registered.pcd", "filepath to output registered point cloud ");
// DEFINE_string(residual_histogram_image_filepath, "[source_filename]_histogram.png", "filepath to output histogram PNG");
// DEFINE_string(fscore_filepath, "[source_filename]_fscore.txt", "filepath to output f-score TXT");

// #ifdef _OPENMP
// DEFINE_bool(no_parallel, false, "run single-threaded");
// #endif

using namespace PRT;

int main(int argc, char *argv[])
{
    int num_ksearch_neighbors = 100;
    double descriptor_radius = 1.0;
    double subsampling_radius = 0.2;
    double consensus_inlier_threshold = 0.2;
    int consensus_max_iterations = 100;
    int icp_max_iterations = 100;
    double icp_max_correspondence_distance = 0.05;
    double residual_threshold = 0.1;
    PointCloudT::Ptr target_cloud;
    PointCloudT::Ptr source_cloud;
    std::string registration_technique = "both";

    // std::string usage_message = "\nUsage:\n\n1. ./point_cloud_registration_tool [options] <source_point_cloud> <target_point_cloud>\n2. ./point_cloud_registration_tool [options] --batch_file <batch_filepath>\n\nA batch file is a two-column CSV. First column: source point cloud filepaths, second column: target point cloud filepaths.\n\nSee --help for optional arguments\n";
    // gflags::SetUsageMessage(usage_message);

    // gflags::ParseCommandLineFlags(&argc, &argv, true);

    // if (FLAGS_descriptor_radius <= 0)
    // {
    //     std::cout << "Error: Descriptor radius must be a positive value." << std::endl;
    //     return -1;
    // }

    // if (FLAGS_subsampling_radius <= 0)
    // {
    //     std::cout << "Error: Subsampling radius must be a positive value." << std::endl;
    //     return -1;
    // }

    // if (FLAGS_consensus_inlier_threshold <= 0)
    // {
    //     std::cout << "Error: Consensus inlier threshold must be a positive value." << std::endl;
    //     return -1;
    // }

    // if (FLAGS_icp_max_correspondence_distance <= 0)
    // {
    //     std::cout << "Error: Maximum correspondence distance must be a positive value." << std::endl;
    //     return -1;
    // }

    // if (FLAGS_residual_threshold <= 0)
    // {
    //     std::cout << "Error: Residual threshold must be a positive value." << std::endl;
    //     return -1;
    // }

    // if (argc < 3 && FLAGS_batch_file.empty())
    // {
    //     std::cout << usage_message << std::endl;
    //     return 0;
    // }

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    FilepairVectorPtr filepairs;
    filepair_t pair;
    pair.sourcefile = argv[1];
    pair.targetfile = argv[2];
    filepairs = FilepairVectorPtr(new FilepairVector());
    filepairs->push_back(pair);

#ifdef _OPENMP
    omp_set_nested(true);
#endif

    for (FilepairVector::size_type i = 0; i < filepairs->size(); ++i)
    {
        PointCloudT::Ptr target_cloud(new PointCloudT);
        target_cloud->clear();
        std::string target_cloud_filepath = filepairs->at(i).targetfile;

        PointCloudT::Ptr source_cloud(new PointCloudT);
        source_cloud->clear();
        std::string source_cloud_filepath = filepairs->at(i).sourcefile;
        bool reads_successful = true;

        if (util::loadPointCloud(source_cloud_filepath, *source_cloud) != 0)
        {
            std::cout << "Invalid input:" << std::endl
                      << source_cloud_filepath << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }

        if (util::loadPointCloud(target_cloud_filepath, *target_cloud) != 0)
        {
            std::cout << "Invalid input:" << std::endl
                      << target_cloud_filepath << std::endl;
            std::cout << "Skipping.." << std::endl;
            reads_successful = false;
        }

        if (!reads_successful)
            continue;

        //Extract prefix for output files
        std::string prefix = util::removeFileExtension(filepairs->at(i).sourcefile);

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
            continue;
        }

        if (util::ensureUniqueFilepath(fscore_filepath) != 0)
        {
            std::cout << "Failed to create f-score file." << std::endl;
            continue;
        }

        if (util::ensureUniqueFilepath(registered_pointcloud_filepath) != 0)
        {
            std::cout << "Failed to create registered PCD file." << std::endl;
            continue;
        }

        if (util::ensureUniqueFilepath(residual_histogram_image_filepath) != 0)
        {
            std::cout << "Failed to create residual histogram file." << std::endl;
            continue;
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
        registrator->setTargetCloud(target_cloud);
        registrator->setSourceCloud(source_cloud);

        //Compute
        registrator->performRegistration(registration_technique);

        //Save Results
        registrator->saveResidualColormapPointCloud(registered_pointcloud_filepath);
        // registrator->saveFinalTransform(transformation_matrix_filepath);
        // registrator->saveFScoreAtThreshold(fscore_filepath, FLAGS_residual_threshold);

        std::cout << "Registration of " << source_cloud_filepath << " finished" << std::endl;
        std::cout << "F-score: " << registrator->getFScoreAtThreshold() << std::endl;
        std::cout << "Saved to: " << registered_pointcloud_filepath << std::endl;

        //Visualization
        Visualizer visualizer("Point Cloud Registration");
        visualizer.setRegistrator(registrator);
        // visualizer.saveHistogramImage(residual_histogram_image_filepath);
        visualizer.visualize();
    }

    return 0;
}
