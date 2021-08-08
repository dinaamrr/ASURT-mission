#include"Bicycle.h"
#include<iostream>
using namespace std;
int main() {
	
	Bicycle bike;
    bike.xc = 0;
	bike.yc = 0;
	bike.theta = 0;
	cout << "initialized bike";
	cout << "parameters before step are all initialized to zero";
	for (int i = 0; i < 20; i++)
	{
		bike.step(2.8, 2.9, 0.23);
		cout << "\n pose after single step: \n pose in x = " << bike.xc
			<< "\n pose in y = " << bike.yc << "\n yaw angle = " << (float) bike.theta;
	}
}
