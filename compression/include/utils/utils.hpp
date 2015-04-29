namespace EXX{
	namespace utils{

		template<typename T>
		T l2_norm( T a ){
			T power = 2.0;
			return std::sqrt( std::pow( a, power ) + std::pow( a, power ) );
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