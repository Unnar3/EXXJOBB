#include <ros/ros.h>

// OTHER
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>


// DEFINITIONS
#define PRINT                   1
#define BUFFER_SIZE             1
#define NODE_NAME               "copy_waypoints"

// using PointT = pcl::PointXYZRGB;
// typedef pcl::PointXYZRGB PointT;
// using PointCloudT = pcl::PointCloud<PointT>;
// using PointNT = pcl::PointNormal;
// using PointNCloudT = pcl::PointCloud<PointNT>;
// using NormalT = pcl::Normal;
// using NormalCloudT = pcl::PointCloud<NormalT>;
// using SurfelT = SurfelType;
// using SurfelCloudT = pcl::PointCloud<SurfelT>;


// Method that takes in a path and creates the directories if they do not exist.
void createDirectories(std::string path){
	boost::filesystem::path dir(path);
    if(boost::filesystem::create_directories(dir)){
        std::cerr << "Directory Created: " << path << std::endl;
    }
};


bool copyDir(
    boost::filesystem::path const & source,
    boost::filesystem::path const & destination
)
{
    namespace fs = boost::filesystem;
    try
    {
        // Check whether the function call is valid
        if(
            !fs::exists(source) ||
            !fs::is_directory(source)
        )
        {
            std::cerr << "Source directory " << source.string()
                << " does not exist or is not a directory." << '\n'
            ;
            return false;
        }
        if(fs::exists(destination))
        {
            std::cerr << "Destination directory " << destination.string()
                << " already exists." << '\n'
            ;
            return false;
        }
        // Create the destination directory
        if(fs::create_directories(destination))
        {
            std::cerr << "Unable to create destination directory"
                << destination.string() << '\n'
            ;
            return false;
        }
    }
    catch(fs::filesystem_error const & e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    // Iterate through the source directory
    for(
        fs::directory_iterator file(source);
        file != fs::directory_iterator(); ++file
    )
    {
        try
        {
            fs::path current(file->path());
            if(fs::is_directory(current))
            {
                // Found directory: Recursion
                if(
                    !copyDir(
                        current,
                        destination / current.filename()
                    )
                )
                {
                    return false;
                }
            }
            else
            {
                // Found file: Copy
                fs::copy_file(
                    current,
                    destination / current.filename()
                );
            }
        }
        catch(fs::filesystem_error const & e)
        {
            std:: cerr << e.what() << '\n';
        }
    }
    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh = ros::NodeHandle("~");

    std::string path = "/media/unnar/HDD/thesis/KTH_longterm_dataset_registered";
    std::string waypoint = "WayPoint5";
    std::string txt_base = "/media/unnar/HDD/thesis/test.txt";
    std::string dataset_base = "/media/unnar/HDD/thesis/Dataset";

    std::vector<std::string> test;
    std::string line;
    std::string string;
    std::ifstream myfile2 (txt_base.c_str());
    int i =0 ;
    if (myfile2.is_open())
        {
        while ( std::getline (myfile2,line) )
        {
            string = line;
            std::size_t t = string.find_last_of("/\\");
            std::string pa = string.substr(0,t);
            std::string p = string.substr(0,t);
            t = pa.find_last_of("/\\");
            p = pa.substr(0,t); // minus room_N

            t = p.find_last_of("/\\");
            std::string patrol = p.substr(t);
            p = p.substr(0,t); // minus room_N

            t = p.find_last_of("/\\");
            std::string date = p.substr(t);
            p = p.substr(0,t); // minus room_N

            std::cout << string << std::endl;
            std::string save_path = dataset_base + date + patrol + "/" + waypoint + "/";

            boost::filesystem::path pA(pa);
            boost::filesystem::path pB(save_path);
            copyDir(pA, pB);
            std::cout << save_path << std::endl;
        }
        myfile2.close();
    }



    ros::Rate loop_rate(10);

    return 0;
}
