#ifndef PCL_H_
#define PCL_H_
#pragma warning(disable:4996)

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/file_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/shot_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/common/transforms.h> 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/board.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/registration/icp.h>
#include <pcl/common/pca.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PCLAdapter.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr Point;
typedef pcl::PointCloud<pcl::Normal>::Ptr Normal;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointNormal;
const int num_hierarchy = 20;//主曲率の階層数

struct PC
{
	float pc1;
	float pc2;
	friend std::ostream& operator << (std::ostream& os, const PC& p);
};

struct PC_hierarchy
{
	float descriptor[num_hierarchy * 2];
	static int descriptorSize() { return num_hierarchy * 2; }
	friend std::ostream& operator << (std::ostream& os, const PC_hierarchy& p);
};

class PCL{
private:
	void read(char *name, Point &data);
	void filtering(Point input, Point &output);
	double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &data);
	void normal_calculation(Point data, Normal &normal);
	void flip_normal(Point Cloud, Normal &Normals);//原点方向に法線をそろえる
	void reverse_normal(Normal input, Normal output);
	void smoothing(Point input, Point &output);
	void Principal_Curvature(Point data, Normal normal, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_cur,float radius);

	//キーポイント
	void keypoints_calculation_iss(Point data, Point &keypoint);//iss
	void keypoints_calculation_Harris(Point data, Point &output);//Harris

	//特徴量
	void feature_calculation_spinimage(Point data, Normal Normals, pcl::PointCloud<pcl::Histogram<153>>::Ptr feature);
	void feature_calculation_shot(Point cloud, Normal normals, Point keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors,int flag_invert);
	void Hierarchy_Principal_Curvature(Point data, Normal normal, pcl::PointCloud<pcl::Histogram<num_hierarchy * 2>>::Ptr feature, float radius_rate, float initial, int num_hierarchy, int flag_invert);

	void convert_xyz_xyzrgb(pcl::PointCloud<pcl::PointXYZ> cloud_xyz, pcl::PointCloud<pcl::PointXYZRGB> &cloud_xyzrgb);
	void segmentate(pcl::PointCloud<pcl::PointXYZRGB> &cloud, double threshould);//平面抽出
	void divide_surface(Point data, Normal Normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output);
	void divide_surface_normal(Point data, Normal Normals, pcl::PointCloud<pcl::PointXYZ>::Ptr &output);

	void concatenate_field(Point data,Normal Normals, PointNormal &cloud_with_Normals);
	void create_mesh(Point data, Normal Normals, PointNormal &cloud_with_Normals, pcl::PolygonMesh &Triangles);

	//一致
	void correspond_spinimages(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::Histogram<153> >::Ptr model_descriptors, pcl::PointCloud<pcl::Histogram<153> >::Ptr scene_descriptors, pcl::CorrespondencesPtr &model_scene_corrs);
	void correspond_shot(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors, pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors, pcl::CorrespondencesPtr model_scene_corrs);
	void correspond_curvature(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr model_curvature, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr scene_curvature, pcl::CorrespondencesPtr model_scene_corrs);
	void correspond_original(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::Histogram<num_hierarchy * 2>>::Ptr model_curvature, pcl::PointCloud<pcl::Histogram<num_hierarchy * 2>>::Ptr scene_curvature, pcl::CorrespondencesPtr model_scene_corrs);
	void remove_incorrect(Point model, Point scene, pcl::CorrespondencesPtr model_scene_corrs, pcl::CorrespondencesPtr refine_corrs);
	void clustering(Point model, Point model_keypoints, Normal model_normals, Point scene, Point scene_keypoints, Normal scene_normals, pcl::CorrespondencesPtr model_scene_corrs, vector < pcl::Correspondences> &clustere_corrs, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans);
	
	int instance_rotate(Point input, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans, Point output);
	void ICP(Point model, Point scene, Point output);
	void registration(Point model,Point scene,vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans, Point output);
	void find_hole(Point input);
	void visualize();
	
	//座標の操作
	void move_center(Point input, float x, float y, float z);
	void initialize_axes(Point input);
	Eigen::Matrix3f eigen_vector;
	Eigen::Vector4f mean;
	Eigen::Vector3f eigen_value;

	//２次元画像を用いた処理
	void processing_with_2D(char *f_name);


public:
	PCL();
	PCL(char *f_name);
	~PCL();
	void Harris3D_SpinImage(char *f_name, char *f_name2);
	void Harris3D_Shot(char *f_name, char *f_name2);
	void Harris3D_Curvature(char *f_name, char *f_name2);
	void Harris3D_Original(char *f_name, char *f_name2);
};


#endif