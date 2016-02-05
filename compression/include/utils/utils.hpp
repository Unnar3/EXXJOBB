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

		template<typename T>
		std::vector<float> vecDiff(T a, T b){
			std::vector<float> d;
			d.push_back(b.x-a.x);
			d.push_back(b.y-a.y);
			d.push_back(b.z-a.z);
			return d;
		}

		template<typename T>
		std::vector<T> vecCross(std::vector<T> a, std::vector<T> b){
			std::vector<T> c;
			c.push_back(  a[1]*b[2] - a[2]*b[1] );
			c.push_back(  a[2]*b[0] - a[0]*b[2] );
			c.push_back(  a[0]*b[1] - a[1]*b[0] );
			return c;
		}

		template<typename T>
		T vecDot(std::vector<T> a, std::vector<T> b){
			return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
		}

		template<typename T>
		std::vector<float> triNorm(T a, T b, T c){
			std::vector<float> d;
			d = vecCross( vecDiff(a,b), vecDiff(a,c) );
			return d;
		}


        template<typename T>
        std::vector<T> vectorIntersection(std::vector<T> v1, std::vector<T> v2){
            std::vector<T> v3;

            std::sort(v1.begin(), v1.end());
            std::sort(v2.begin(), v2.end());

            std::set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(),std::back_inserter(v3));

            return v3;
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
