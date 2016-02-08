#include <ros/ros.h>
#include <bitset>
#include <math.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace EXX{
	namespace utils{

		struct OBBcube{
		    Eigen::Vector3f position;
		    Eigen::Matrix3f rot;
		    float x;
		    float y;
		    float z;
		};


		template<typename T>
		T l2_norm( T a );

		template<typename T>
		T sigmoid( T a, T b );

		template<typename T>
		T fast_sigmoid( T a, T b );

		template<typename T>
		std::vector<float> vecDiff(T a, T b);

		template<typename T>
		std::vector<T> vecCross(std::vector<T> a, std::vector<T> b);

		template<typename T>
		T vecDot(std::vector<T> a, std::vector<T> b);

		template<typename T>
		std::vector<float> triNorm(T a, T b, T c);

        template<typename T>
        std::vector<T> vectorIntersection(std::vector<T> a, std::vector<T> b);

		template<int T>
		std::bitset<T> shiftRight( std::bitset<T> x ){
			return ( x << 1 | x >> (T-1) );
		}

		template<int T>
		std::bitset<T> shiftLeft( std::bitset<T> x ){
			return ( x >> 1 | x << (T-1) );
		}



        // Calculates the minimum oriented bounding box with relation to x and y directions, that is the bounding
        // box is aligned to the z axis.
        // Input:
        //  clouds, vector containing PointClouds where we want to extract the bounding box.
        // Output:
        //  obbvec, vector containing oriented bounding boxes for all PointClouds.
        void getOBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, OBBcube &obb);

	}

	namespace params{
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh);
	}
}
#include "utils.hpp"
