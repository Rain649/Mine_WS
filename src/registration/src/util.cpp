#include "util.hpp"

// DEFINE_bool(verbose, false, "enable verbosity");

namespace util
{
    static std::random_device rd;
    static std::mt19937 rng(rd());
    std::uniform_int_distribution<> dist(0, 100);

    /**
     Print 4x4 in human readable form
     */
    void print4x4Matrix(const Eigen::Matrix4f &matrix)
    {
        printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
        printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
        printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
        printf("t = < %6.3f, %6.3f, %6.3f >\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
    }

    /**
     Uniformly randomly sample number in the range [0,1]
     */
    double uniformRandom()
    {
        return (double)dist(rng) / 100.0;
    }

    /**
     Load point cloud from disk
     */
    int loadPointCloud(std::string filepath, pcl::PointCloud<PointT> &cloud)
    {

        if (filepath.empty())
            return -1;

        std::string file_extension = extractFileExtension(filepath);
        std::transform(file_extension.begin(), file_extension.end(), file_extension.begin(), ::tolower); //Convert to lowercase

        if (file_extension.compare(".pcd") == 0)
        {
            if (pcl::io::loadPCDFile<PointT>(filepath, cloud) == -1)
            {
                PCL_ERROR("Couldn't read pcd file\n"); // //不带路径的格式【只是把路径删掉即可】
                return -1;
            }
        }
        else
        {
            return -1;
        }

        // if (FLAGS_verbose)
        // {
        //     std::cout << "Loaded point cloud with " << cloud.size() << " points:" << std::endl;
        //     std::cout << filepath << std::endl;
        // }

        return 0;
    }

    /**
     Write XYZ point cloud to disk in PLY format
     */
    int writePointCloudToPLY(std::string filepath, PointCloudT &cloud)
    {

        if (filepath.empty())
            return -1;

        std::string file_extension = extractFileExtension(filepath);
        std::transform(file_extension.begin(), file_extension.end(), file_extension.begin(), ::tolower); //Convert to lowercase

        if (file_extension.compare(".ply") != 0)
            filepath.append(".ply");

        if (pcl::io::savePLYFile(filepath, cloud) < 0)
        {
            std::cout << "Error saving cloud " << filepath << std::endl;
            return -1;
        }

        return 0;
    }

    /**
     Write XYZRGBA point cloud to disk in PCD format
     */
    int writePointCloudToPCD(std::string filepath, PointCloudRGBT &cloud)
    {
        if (filepath.empty())
            return -1;

        std::string file_extension = extractFileExtension(filepath);
        std::transform(file_extension.begin(), file_extension.end(), file_extension.begin(), ::tolower); //Convert to lowercase

        if (file_extension.compare(".pcd") != 0)
            filepath.append(".pcd");

        if (pcl::io::savePCDFileASCII(filepath, cloud) < 0)
        {
            std::cout << "Error saving cloud " << filepath << std::endl;
            return -1;
        }

        return 0;
    }

    /**
     Write XYZRGBA point cloud to disk in PLY  format
     */
    int writePointCloudToPLY(std::string filepath, PointCloudRGBT &cloud)
    {
        if (filepath.empty())
            return -1;

        std::string file_extension = extractFileExtension(filepath);
        std::transform(file_extension.begin(), file_extension.end(), file_extension.begin(), ::tolower); //Convert to lowercase

        if (file_extension.compare(".ply") != 0)
            filepath.append(".ply");

        if (pcl::io::savePLYFile(filepath, cloud) < 0)
        {
            std::cout << "Error saving cloud " << filepath << std::endl;
            return -1;
        }

        return 0;
    }

    /**
     Transform a PointCloud
     */
    void transformPointCloud(PointCloudT::Ptr &cloud, double alpha, double beta, double gamma, double tx, double ty, double tz)
    {

        Eigen::Affine3f T = Eigen::Affine3f::Identity();

        T.translation() << tx, ty, tz;
        T.rotate(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
        T.rotate(Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY()));
        T.rotate(Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ()));

        // if (FLAGS_verbose)
        // {
        //     std::cout << "Applying Transformation:" << std::endl;
        //     util::print4x4Matrix(T.matrix());
        // }

        pcl::transformPointCloud(*cloud, *cloud, T);
    }

    /**
     Transform a PointCloud with randomly chosen parameters
     Angles in the range [0,2pi]
     Translation in the range [-1,1]
     */
    void randomTransformPointCloud(PointCloudT::Ptr &cloud)
    {

        double alpha = 2.0 * M_PI * uniformRandom();
        double beta = 2.0 * M_PI * uniformRandom();
        double gamma = 2.0 * M_PI * uniformRandom();
        double tx = 2.0 * uniformRandom() - 1.0;
        double ty = 2.0 * uniformRandom() - 1.0;
        double tz = 2.0 * uniformRandom() - 1.0;

        transformPointCloud(cloud, alpha, beta, gamma, tx, ty, tz);
    }

    /**
     Converts HSV color to RGB
     Everything in [0,1] range except h in [0,360] range
     */
    void hsv2rgb(double &r, double &g, double &b, double &h, double &s, double &v)
    {
        hsvrgb::HSVtoRGB(r, g, b, h, s, v);
    }

    /**
     Converts RGB color to HSV
     Everything in [0,1] range except h in [0,360] range
     */
    void rgb2hsv(double &r, double &g, double &b, double &h, double &s, double &v)
    {
        hsvrgb::HSVtoRGB(r, g, b, h, s, v);
    }

    /**
     Return the last l characters of the input string
     */
    std::string stringTail(const std::string s, const size_t l)
    {
        if (l >= s.size())
        {
            return s;
        }
        return s.substr(s.size() - l);
    }

    /**
     Returns a vector with the largest element being 'min'
     */
    void vectorMin(DoubleVectorPtr &v, double min)
    {

        for (DoubleVector::iterator it = v->begin(); it != v->end(); it++)
            *it = std::min(*it, min);
    }

    /**
     Rescales values in the vector to a [0,1] range
     */
    void vectorRescale(DoubleVectorPtr &v, double max_element = -1)
    {

        if (max_element == -1)
            max_element = *std::max_element(v->begin(), v->end());

        for (DoubleVector::iterator it = v->begin(); it != v->end(); it++)
            *it /= max_element;
    }

    /**
     Remove all elements in vector above 'threshold'
     */
    void removeElementsAboveThreshold(DoubleVectorPtr &v, double threshold)
    {
        v->erase(std::remove_if(v->begin(), v->end(), boost::bind(std::greater<double>(), _1, threshold)), v->end());
    }

    /**
     Writes an Eigen matrix to disk
     */
    int writeMatrixToFile(const Eigen::MatrixXf &matrix, const std::string filepath)
    {

        Eigen::IOFormat format(Eigen::FullPrecision, 0, ",", "\n", "", "", "", "");
        std::ofstream outfile(filepath);

        if (outfile.is_open())
        {
            outfile << matrix.format(format);
            return 0;
        }
        return -1;
    }

    /**
     Reads in an ascii file containing mesh pairs to be registered
     Expects two filepaths per line, comma separated without spaces
     First filepath is to the target point cloud, second is to the source point cloud
     */
    FilepairVectorPtr readBatchProcessingFile(std::string filepath)
    {

        FilepairVectorPtr filepairs(new FilepairVector());
        std::ifstream batchfile(filepath);

        if (!batchfile.is_open())
            return filepairs;

        std::string line;
        while (std::getline(batchfile, line))
        {
            if (line[0] == '#')
                continue;

            int comma_pos = line.find(',');

            filepair_t pair;
            pair.sourcefile = line.substr(0, comma_pos);
            pair.targetfile = line.substr(comma_pos + 1, std::string::npos);
            filepairs->push_back(pair);
        }

        return filepairs;
    }

    std::string removeFileExtension(const std::string &filepath)
    {

        std::size_t ext_begin_pos = filepath.find_last_of('.');
        return filepath.substr(0, ext_begin_pos);
    }

    std::string extractFileExtension(const std::string &filepath)
    {

        std::size_t ext_begin_pos = filepath.find_last_of('.');
        if (ext_begin_pos == std::string::npos)
            return "";
        return filepath.substr(ext_begin_pos, std::string::npos);
    }

    int ensureUniqueFilepath(std::string &filepath)
    {
        if (!fileExists(filepath))
            return 0;

        std::string filepath_without_ext = removeFileExtension(filepath);
        std::string filepath_ext = extractFileExtension(filepath);

        int i = 1;
        while (i < 100)
        {
            std::stringstream ss;
            ss << filepath_without_ext << "_" << i++ << filepath_ext;
            if (!fileExists(ss.str()))
            {
                filepath = ss.str();
                return 0;
            }
        }

        return -1;
    }

    bool fileExists(const std::string &filepath)
    {
        struct stat buffer;
        return (stat(filepath.c_str(), &buffer) == 0);
    }

    void writeStringToFile(const std::string &filepath, const std::string s)
    {

        std::ofstream out(filepath);
        out << s;
        out.close();
    }
}
