#include "PCLAdapter.h"
#include "PCL.h"
#include"parameter.h"
#include "Corresponding_point.h"
using namespace std;

extern Parameter parameter;

int main(int argc, char** argv)
{
	/*Corresponding_point *correponding_point;
	correponding_point = new Corresponding_point();
	correponding_point->image_view(parameter.file_name1, parameter.file_name2);
	delete correponding_point;*/


	PCL *pcl;
	pcl = new PCL(parameter.ply_file);
	delete  pcl;

	return 0;
}
