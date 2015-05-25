#include <iostream>
#include <algorithm>
#include <vector>
int main ()
{
  	std::cout << "Hello World! " ;
  	std::cout << "I'm a C++ program" << std::endl;;
  
 	std::vector<int> a = {1,2,4,3,6,4,5,3,4};
 	
 	auto lambdafunc = [](int value){ 
 		std::cout << value*10-1 << " ";
 	};

 	int thresh = 3;
 	int count = std::count_if(a.begin(), a.end(), 
 		[thresh](int number){ return (number > thresh); });

 	std::cout << "a: ";
 	std::for_each(a.begin(), a.end(), [](int number){std::cout << " " << --number << " ";});
 	std::cout << "" << std::endl;
 	std::cout << "a: ";
 	std::for_each(a.begin(), a.end(), lambdafunc);
 	std::cout << "" << std::endl;
 	std::cout << "threshold: " << thresh << std::endl;
 	std::cout << "count: " << count << std::endl;

  	return 0;
}