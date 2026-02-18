#include <iostream>
#include <vector>

struct point {
	float x, y;
};



float checkVec(float ax, float ay, float bx, float by, float px, float py) {

	float d = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
	return d;
}




int main() {
	std::cout << "Hello world\n";
	std::vector<point> pValues = {
		{52.4067821,5.9823412},
		{52.4012395,5.9914328},
		{52.4129857,5.9442185},
		{52.3976543,5.9578124},
		{52.4141120,5.9785632},
		{52.3934512,5.9337210},
		{52.4098723,5.9965121},
		{52.4001239,5.9458127},
		{52.4156734,5.9781239},
		{52.3952841,5.9645123},
		{52.4073215,5.9887124},
		{52.3987654,5.9309812},
		{52.4124391,5.9572234},
		{52.4041122,5.9911238},
		{52.4105678,5.9387125},
		{52.3939876,5.9456123},
		{52.4132459,5.9821110},
		{52.3998765,5.9764321},
		{52.4081234,5.9587124},
		{52.4019871,5.9901235},
		{52.4148762,5.9632123},
		{52.3965432,5.9309871},
		{52.4114321,5.9887654},
		{52.4032123,5.9521112},
		{52.4098765,5.9743210},
		{52.3947653,5.9398721},
		{52.4129876,5.9832112},
		{52.4004321,5.9601234},
		{52.4076543,5.9973211},
		{52.3959871,5.9447654},
		{52.411,5.9895},


	};
	for (int i = 0; i < pValues.size(); i++) {
		point neededPoint = pValues.at(i);
		float crossproduct = checkVec(52.392015, 5.925107, 52.416838, 6.009521, neededPoint.x, neededPoint.y); 
		bool isWithinBounds = (crossproduct > 0);
		std::cout << neededPoint.x << " ; " << neededPoint.y << " ; ";
		std::cout << crossproduct << " ; ";
		std::cout << isWithinBounds << "\n";
	}	
	return 0;
}
