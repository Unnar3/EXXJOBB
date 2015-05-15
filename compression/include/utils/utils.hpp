namespace EXX{
	namespace utils{

		template<typename T>
		T l2_norm( T a ){
			T power = 2.0;
			return std::sqrt( std::pow( a, power ) + std::pow( a, power ) );
		}

		template<typename T>
		T sigmoid( T ratio, T slope ){
			return 1 / ( 1 + std::exp(-slope*(ratio - 0.5)));
		}

		template<typename T>
		T fast_sigmoid( T ratio, T slope ){
			return (slope+1)/2 * (2*ratio-1) / (1+std::abs(slope*(2*ratio+1))) + 0.5;
		}
	}

	namespace params{
		template<typename T>
		T loadParam( std::string name, ros::NodeHandle &nh){
			T param;
			if (nh.hasParam( name )){
			    nh.getParam( name, param);
			    return param;
			} else {
			    std::cout << "Param " << name << " does not exist." << std::endl;
			    exit(0);
			}
	    }
	}
}