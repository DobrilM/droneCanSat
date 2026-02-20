#pragma once

struct bmpData {
	float temperature;
	float pressure;
	float altitude;
};
void bmpSetup();

bmpData bmpRead();
