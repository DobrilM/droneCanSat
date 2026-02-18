#include <iostream>
#include <vector>
#include <fstream>


struct point {
	float x, y;
};

float checkVec(point a, point b, point p) {

	float d = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
	return d;
} 

std::vector<float> checkGeozone(point p, std::vector<point>& c){
	std::vector<float> result;
	
	result.push_back(checkVec(c.at(0), c.at(1), p));
	result.push_back(checkVec(c.at(1), c.at(2), p));
	result.push_back(checkVec(c.at(2), c.at(3), p));
	result.push_back(checkVec(c.at(3), c.at(0), p));

	return result;
}
float randomFloat(float min, float max)
{
   float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;  
    return (random*range) + min;
}

int main(int argc, char* argv[]) {



	srand(time(nullptr));
	point min = {-10, -10};
	point max = {10, 10};
	std::vector<point> corners = {
		{-8.1234321, -2.1987425},
		{9.6859451, -6.2314984},
		{5.348712,5.1725622},
		{-7.5687023, 0}
	};

	if (argc < 2) {
		std::cout << "Too little arguments";
		return 0;
	} else if (argc > 2) {
		std::cout << "Too many arguments";
		return 0;
	}

	std::cout << "Hello world\n";
	int testCount(std::stoi(argv[1]));
	for (int i = 0; i < testCount; i++) {
		point p = {randomFloat(min.x, max.x), randomFloat(min.y, max.y)};

		std::cout << p.x << " ; " << p.y << " ; ";
		std::vector<float> result = checkGeozone(p, corners);
		std::cout << result.at(0) << " ; ";
		
		std::cout << result.at(1) << " ; ";
		std::cout << result.at(2) << " ; ";
		std::cout << result.at(3) << " ; ";

		if(result.at(0) > 0 && result.at(1) > 0 && result.at(2) > 0 && result.at(3) > 0 ) {
			std::cout << true << " ; ";
		} else {
			std::cout << false << " ; ";
		}
		std::cout << "\n";
	}
	return 0;

}
