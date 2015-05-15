#include <ros/ros.h>
#include <bitset>
#include <math.h>

namespace EXX{
	namespace utils{

		template<typename T>
		T l2_norm( T a );

		template<typename T>
		T sigmoid( T a, T b );

		template<typename T>
		T fast_sigmoid( T a, T b );

		template<int T>
		std::bitset<T> shiftRight( std::bitset<T> x ){
			return ( x << 1 | x >> (T-1) );
		}

		template<int T>
		std::bitset<T> shiftLeft( std::bitset<T> x ){
			return ( x >> 1 | x << (T-1) );
		}


	}

	namespace params{
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh);
	}
}
#include "utils.hpp"