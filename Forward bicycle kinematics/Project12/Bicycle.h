#pragma once
#include <math.h>

class Bicycle
{
public:
	double xc;
	double yc;
	double theta;
	float delta = 0;
	float beta = 0;
	int L = 2;
	float lr = 1.2;
	float sample_time = 0.01;
public:
	void reset() {
		 xc = 0;
		 yc = 0;
		 theta = 0;
		 beta = 0;
		 delta = 0;

	}
	void step(float wr,float wl,float delta) {
		double r = atan(delta) / L;
		double v = r * (wr + wl) / 2;
		double xc_dot = v * cos(theta + beta);
		double yc_dot = v * sin(theta + beta);
		double theta_dot = (v / L) * (cos(beta) * tan(delta));
		beta = atan(lr * tan(delta) / L);
		xc += xc_dot * sample_time;
		yc += yc_dot * sample_time;
		theta += theta_dot * sample_time;
		delta = delta;
	}
};



