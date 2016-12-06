#ifndef PARAMETER_H
#define PARAMETER_H

#include<iostream>
using namespace std;
class Parameter{
private:

public:
	Parameter();
	~Parameter();

	//corresponding_point.cpp
	string file_name1;
	string file_name2;
	int DPM_displacement;
	int DPM_different;
	double Threshold_error;
	double hole_point_size;
	double ransac;
	int crrespond_count;

	//PCL.cpp
	char *ply_file;
	float smooth;
	float theta[3];
	float view_point[3];
	float flip_n[3];

};

#endif