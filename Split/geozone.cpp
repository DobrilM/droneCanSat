//#include "config.h"
#include "geozone.h"

point borders[4] = {
	{52.402008, 5.924794},
	{52.424292, 5.966989},
	{52.414117, 5.979726},
	{52.395261, 5.927807},
};



float checkVec(point a, point b, float latitude, float longitude){
	
float d = (b.x - a.x) * (longitude - a.y) - (b.y - a.y) * (latitude - a.x);

return d;
}

bool geozoneCheck(float latitude, float longitude) {
	int totalCheck = 0;	
	for (int i = 0; i < (sizeof(borders)/sizeof(borders[0]))-1; i++) {
		totalCheck += (checkVec(borders[i], borders[i+1], latitude, longitude)>0);
	}
	totalCheck+= (checkVec(borders[3], borders[0], latitude, longitude) > 0);
	return (totalCheck == (sizeof(borders)/sizeof(borders[0])));
}
