#include "parameter.h"

Parameter::Parameter(){
	//find_hole_pcl_624.png & find_hole_pcl_623.png
	/*file_name1 = "find_hole_pcl_624.png";
	file_name2 = "find_hole_pcl_623.png";
	DPM_displacement = 1;
	DPM_different = 3;
	Threshold_error = 4.0;
	hole_point_size =10.0;
	ransac = 1;
	crrespond_count = 5;*/

	//find_hole_pcl_624.png & find_hole_pcl_623.png
	file_name1 = "find_hole_pcl_623.png";
	file_name2 = "find2_hole_pcl_624.png";
	DPM_displacement = 2;
	DPM_different = 3;
	Threshold_error = 10;
	hole_point_size = 10.0;
	ransac = 3;
	crrespond_count = 5;

	//test.png & 90_test.png
	/*file_name1 = "test2.png";
	file_name2 = "test2.png";
	DPM_displacement = 2;
	DPM_different = 5;
	Threshold_error = 0.2;
	hole_point_size = 10.0;
	ransac = 1;
	crrespond_count = 5;*/

	/*PCL.cpp*/
	ply_file = "merged/625Merged_d12_fillhole.ply";
	smooth = 0.08;
	theta[0] = 0.3;
	theta[1] = 0.7;
	theta[2] = 0.7;
	view_point[0] = 0;
	view_point[1] = 0;
	view_point[2] = 4.5;
	flip_n[0] = 0;
	flip_n[1] = 0;
	flip_n[2] = 4.5;
}

Parameter::~Parameter(){

}