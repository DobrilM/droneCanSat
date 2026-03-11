#pragma once

struct point {
	float x, y;
	    point(float x_, float y_) : x(x_), y(y_) {}
};
float checkVec(point a, point b, float latitude, float longitude);

bool geozoneCheck(float latitude, float longitude);
