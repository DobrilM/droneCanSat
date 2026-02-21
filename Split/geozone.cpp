//#include "config.h"
#include "geozone.h"

point polygonPoints[4] = {
	{52.39450997696698f, 5.925463407495726f},
	{52.413715862246015f, 5.985669372917519f},
	{52.427675919343486f, 5.966253319200584f},
	{52.39990173031024f, 5.9208956953653535f}
};



float checkVec(point a, point b, float latitude, float longitude){
	
float d = (b.x - a.x) * (longitude - a.y) - (b.y - a.y) * (latitude - a.x);

return d;
}

bool geozoneCheck(float latitude, float longitude) {
	int totalCheck = 0;	
	for (int i = 0; i < (sizeof(polygonPoints)/sizeof(polygonPoints[0]))-1; i++) {
		totalCheck += (checkVec(polygonPoints[i], polygonPoints[i+1], latitude, longitude)>0);
	}
	totalCheck+= (checkVec(polygonPoints[3], polygonPoints[0], latitude, longitude) > 0);
	return (totalCheck == (sizeof(polygonPoints)/sizeof(polygonPoints[0])));
}
