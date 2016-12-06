#include "PCL.h"
#include "parameter.h"

using namespace std;
extern const int num_hierarchy;

pcl::PolygonMesh mesh;
Point cloud(new pcl::PointCloud<pcl::PointXYZ>);
Point cloud2(new pcl::PointCloud<pcl::PointXYZ>);
Point cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
Point cloud_smooth(new pcl::PointCloud<pcl::PointXYZ>);
Point cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

//ポイントクラウドの法線
Normal normals(new pcl::PointCloud<pcl::Normal>);
Normal normals2(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);


//キーポイントの法線
Normal normals_keypoints(new pcl::PointCloud<pcl::Normal>);
Normal normals_keypoints2(new pcl::PointCloud<pcl::Normal>);

//主曲率
pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>());
pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures2(new pcl::PointCloud<pcl::PrincipalCurvatures>());

//キーポイント
Point keypoints_iss(new pcl::PointCloud<pcl::PointXYZ>);//iss
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_Harris(new pcl::PointCloud<pcl::PointXYZ>);//Harris
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_Harris2(new pcl::PointCloud<pcl::PointXYZ>);//Harris

//特徴量
pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images(new pcl::PointCloud<pcl::Histogram<153>>);
pcl::PointCloud<pcl::Histogram<153>>::Ptr spin_images2(new pcl::PointCloud<pcl::Histogram<153>>);
pcl::PointCloud<pcl::SHOT352>::Ptr shot(new pcl::PointCloud<pcl::SHOT352>());
pcl::PointCloud<pcl::SHOT352>::Ptr shot2(new pcl::PointCloud<pcl::SHOT352>());
pcl::PointCloud<pcl::Histogram<num_hierarchy*2>>::Ptr Hierarchy_PC(new pcl::PointCloud<pcl::Histogram<num_hierarchy*2>>());
pcl::PointCloud<pcl::Histogram<num_hierarchy*2>>::Ptr Hierarchy_PC2(new pcl::PointCloud<pcl::Histogram<num_hierarchy*2>>());


//特徴点の一致
pcl::CorrespondencesPtr corrs(new pcl::Correspondences());
pcl::CorrespondencesPtr refine_corrs(new pcl::Correspondences());
std::vector < pcl::Correspondences > clustered_corrs;
std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;

//レジストレーション
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_registrated;

extern Parameter parameter;

PCL::PCL(){
	
	/*3次元特徴量による一致*/
	//Harris3D_SpinImage("624_cut.ply"2,"623_cut.ply");

	//Harris3D_Shot("ZF1_20160719_623_simMer.ply", "ZF1_20160719_624_simMer.ply");
	//Harris3D_Shot("bunny.ply", "bunny_moved.ply");
	//Harris3D_Shot("624_cut.ply", "624_cut_moved.ply");
	//Harris3D_Shot("624_cut.ply", "623_cut.ply");

	//Harris3D_Curvature("bunny.ply", "bunny_moved.ply");
	//Harris3D_Curvature("624_cut.ply","624_cut_moved.ply");
	//Harris3D_Curvature("624_cut.ply", "623_cut.ply");

	//Harris3D_Original("bunny.ply", "bunny_moved.ply");
	//Harris3D_Original("624_cut.ply","624_cut_moved.ply");
	//Harris3D_Original("624_cut.ply", "623_cut.ply");

	/*面の分割*/
	//divide_surface(cloud, normals, cloud_rgb);
	//visualize();

	/*二次元画像との処理*/
	//processing_with_2D("merged/624Merged_d11_fillhole.ply");
}

PCL::PCL(char *f_name){
	/*二次元画像との処理*/
	processing_with_2D(f_name);
}

PCL::~PCL(){

}


void PCL::read(char *name, Point &data){
	pcl::io::loadPLYFile(name, *data);
	cout << "read : " << name << endl;
}

double PCL::computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &data)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void PCL::filtering(Point input, Point &output){
	//フィルタリング
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(input);
	//sor.setMeanK(50);
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.5);
	sor.filter(*output);
	cout << "filtering_do" << endl;
}

void PCL::normal_calculation(Point data, Normal &normal){
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointXYZ>);
	tree_normal->setInputCloud(data);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	n.setInputCloud(data);
	n.setSearchMethod(tree_normal);
	n.setRadiusSearch(0.012);
	//n.setKSearch(100);
	//n.setKSearch(40);
	n.compute(*normal);
	cout << "normal_calculation" << endl;
}

void PCL::flip_normal(Point Cloud,Normal &Normals){
	for (int i = 0; i < Normals->points.size(); i++){
		Eigen::Vector4f normal_flip(Normals->points[i].normal[0], Normals->points[i].normal[1], Normals->points[i].normal[2], Normals->points[i].normal[3]);
		flipNormalTowardsViewpoint(Cloud->points[i], parameter.view_point[0], parameter.view_point[1], parameter.view_point[2], normal_flip);
		Normals->points[i].normal[0] = normal_flip(0);
		Normals->points[i].normal[1] = normal_flip(1);
		Normals->points[i].normal[2] = normal_flip(2);
		Normals->points[i].normal[3] = normal_flip(3);
		/*if (Normals->points[i].normal[0] != normal_flip(0)){
			cout << "flip" << endl;
		}*/
	}
}


void PCL::keypoints_calculation_iss(Point data, Point &keypoint){
	float model_resolution = static_cast<float> (computeCloudResolution(cloud));
	double iss_salient_radius_ = 2 * model_resolution;
	double iss_non_max_radius_ = 4 * model_resolution;
	double iss_gamma_21_(0.975);
	double iss_gamma_32_(0.975);
	double iss_min_neighbors_(5);
	int iss_threads_(4);
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_keypoint(new pcl::search::KdTree<pcl::PointXYZ>);
	iss_detector.setSearchMethod(tree_keypoint);
	iss_detector.setSalientRadius(iss_salient_radius_);
	iss_detector.setNonMaxRadius(iss_non_max_radius_);
	iss_detector.setThreshold21(iss_gamma_21_);
	iss_detector.setThreshold32(iss_gamma_32_);
	iss_detector.setMinNeighbors(iss_min_neighbors_);
	iss_detector.setNumberOfThreads(iss_threads_);
	iss_detector.setInputCloud(data);
	iss_detector.compute(*keypoint);

	cout << "keypoints_calculation_iss" << endl;
}

void PCL::keypoints_calculation_Harris(Point data, Point &output){
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> detector;
	detector.setNonMaxSupression(true);
	detector.setRadius(0.007);
	//detector.setRadiusSearch (100);
	detector.setNumberOfThreads(2);
	detector.setInputCloud(data);
	detector.compute(*keypoints);

	pcl::PointXYZ tmp;
	double max = 0, min = 0;
	for (pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i != keypoints->end(); i++){
		tmp = pcl::PointXYZ((*i).x, (*i).y, (*i).z);
		if ((*i).intensity>max){
			std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
			max = (*i).intensity;
		}
		if ((*i).intensity<min){
			min = (*i).intensity;
		}
		output->push_back(tmp);
	}

	cout << "keypoints_calculation_Harris : " << output->points.size() << endl;
}

void PCL::feature_calculation_spinimage(Point data, Normal Normals, pcl::PointCloud<pcl::Histogram<153>>::Ptr feature){
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>> spin_image_descriptor(8, 0.5, 16);
	
	spin_image_descriptor.setInputCloud(data);
	spin_image_descriptor.setInputNormals(Normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	spin_image_descriptor.setSearchMethod(kdtree);
	spin_image_descriptor.setRadiusSearch(0.6);
	spin_image_descriptor.compute(*feature);
	cout << "calculate spinimage" << endl;
}

void PCL::feature_calculation_shot(Point cloud, Normal normals, Point keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr descriptors,int flag_invert){
	pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
	float descr_rad_(0.05f);
	float rf_rad_(0.05f);
	pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf(new pcl::PointCloud<pcl::ReferenceFrame>());
	pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
	rf_est.setFindHoles(true);
	rf_est.setRadiusSearch(rf_rad_);
	rf_est.setInputCloud(cloud);
	rf_est.setInputNormals(normals);
	rf_est.setSearchSurface(cloud);
	rf_est.compute(*rf);

	if (flag_invert){
		for (int i = 0; i < cloud->points.size(); i++){
			/*for (int j = 0; j < 3; j++){
				rf->points[i].x_axis[j] *= -1;
				rf->points[i].y_axis[j] *= -1;
				rf->points[i].z_axis[j] *= -1;
			}*/
		}
	}

	descr_est.setRadiusSearch(descr_rad_);
	descr_est.setNumberOfThreads(2);
	descr_est.setInputCloud(keypoints);
	descr_est.setInputNormals(normals);
	descr_est.setSearchSurface(cloud);
	descr_est.setInputReferenceFrames(rf);
	descr_est.compute(*descriptors);

	if (flag_invert){
		for (int i = 0; i < cloud->points.size(); i++){
			for (int j = 0; j < 352; j++){
				descriptors->points[i].descriptor[j] *= -1;
			}
		}
	}
	cout << "shot_calculate : "<<descriptors->size() << endl;
}

void PCL::Principal_Curvature(Point data, Normal normal,pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_cur,float radius){
	
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

	principal_curvatures_estimation.setInputCloud(data);

	principal_curvatures_estimation.setInputNormals(normal);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	principal_curvatures_estimation.setSearchMethod(tree);
	//principal_curvatures_estimation.setRadiusSearch(0.7);
	//principal_curvatures_estimation.setRadiusSearch(0.65);
	//principal_curvatures_estimation.setRadiusSearch(0.45);
	principal_curvatures_estimation.setRadiusSearch(radius);
	cout << "radius : " << radius << endl;

	principal_curvatures_estimation.compute(*principal_cur);

	cout << "output points.size (): " << principal_cur->points.size() << std::endl;

	// Display and retrieve the shape context descriptor vector for the 0th point.
	//pcl::PrincipalCurvatures descriptor = principal_cur->points[0];
	//cout <<"1 : "<< descriptor.pc1 << endl;
	//cout << "2 : " << descriptor.pc2 << endl;
	//cout << "z : " << descriptor.principal_curvature_z << endl;
}

void PCL::Hierarchy_Principal_Curvature(Point data, Normal normal, pcl::PointCloud<pcl::Histogram<num_hierarchy*2>>::Ptr feature, float radius_rate, float initial, int num_hierarchy, int flag_invert){
	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pc(new pcl::PointCloud<pcl::PrincipalCurvatures>());
	feature->points.resize(data->points.size());
	
	for (int i = 0; i < num_hierarchy; i++){
		
		Principal_Curvature(data, normal, pc, initial + (double)i*radius_rate);
		
		if (flag_invert){
			for (int j = 0; j < pc->points.size(); j++){
				feature->points[j].histogram[2*i] = -1.0*pc->points[j].pc2;
				feature->points[j].histogram[2*i+1] = -1.0*pc->points[j].pc1;
				/*feature->points[j].pc1.push_back(tmp.pc1);
				feature->points[j].pc2.push_back(tmp.pc2);*/
				//cout << "feature : " << i << ":" << j << ":" << feature->points[j].pc1[i] << endl;
			}
		} else{
			for (int j = 0; j < pc->points.size(); j++){
				feature->points[j].histogram[2*i] = pc->points[j].pc1;
				feature->points[j].histogram[2*i+1] = pc->points[j].pc2;
				/*feature->points[j].pc1.push_back(tmp.pc1);
				feature->points[j].pc2.push_back(tmp.pc2);*/
				//cout << "feature : " << i << ":" << j << ":" << feature->points[j].pc1[i] << endl;
			}
			//cout << "size : " << pc->points.size() << endl;
		}

		//if (i >= 2){
		//	for (int j = 0; j < pc->points.size(); j++){
		//		/*feature->points[j].histogram[2 * i] = 0;
		//		feature->points[j].histogram[2 * i + 1] = 0;*/
		//		//cout << feature->points[j].histogram[2 * i + 1] << endl;

		//	}
		//}
	}

	cout << "Hierarchy Principal Curvature" << endl;
}

void PCL::reverse_normal(Normal input, Normal output){
	for (int i = 0; i < input->points.size(); i++){
		output->points[i].normal_x = -1.0*input->points[i].normal_x;
		output->points[i].normal_y = -1.0*input->points[i].normal_y;
		output->points[i].normal_z = -1.0*input->points[i].normal_z;
		/*output->points[i].data_c[0] = -1.0*input->points[i].data_c[0];
		output->points[i].data_c[1] = -1.0*input->points[i].data_c[1];
		output->points[i].data_c[2] = -1.0*input->points[i].data_c[2];
		output->points[i].data_c[3] = -1.0*input->points[i].data_c[3];
		output->points[i].curvature = -1.0*input->points[i].curvature;
		output->points[i].data_n[0] = -1.0*input->points[i].data_n[0];
		output->points[i].data_n[1] = -1.0*input->points[i].data_n[1];
		output->points[i].data_n[2] = -1.0*input->points[i].data_n[2];
		output->points[i].data_n[3] = -1.0*input->points[i].data_n[3];*/

	}
}


void PCL::smoothing(Point input,Point &output){
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//Kdtreeの作成
	pcl::PointCloud<pcl::PointNormal> mls_points;//出力する点群の格納場所を作成
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals(true);//法線の計算を行うかどうか

	// 各パラメーターの設定
	mls.setInputCloud(input);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.01);
	//mls.setSearchRadius(0.03);
	mls.setSearchRadius(parameter.smooth);

	mls.process(mls_points);//再構築

	copyPointCloud((input, mls_points), *output);
	cout << "smoothing" << endl;
}

void PCL::concatenate_field(Point data, Normal Normals, PointNormal &cloud_with_Normals){
	pcl::concatenateFields(*data, *Normals, *cloud_with_Normals);
	cout << "concatenate_field" << endl;
}

void PCL::create_mesh(Point data, Normal Normals, PointNormal &cloud_with_Normals, pcl::PolygonMesh &Triangles){
	concatenate_field(data,Normals,cloud_with_Normals);

	//k分木作成
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree_mesh(new pcl::search::KdTree<pcl::PointNormal>);
	tree_mesh->setInputCloud(cloud_with_Normals);

	//物体の初期処理
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	gp3.setSearchRadius(0.025);//サーチの距離設定

	//パラメータの設定
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	//メッシュ作成
	gp3.setInputCloud(cloud_with_Normals);
	gp3.setSearchMethod(tree_mesh);
	gp3.reconstruct(Triangles);
	cout << "create_mesh" << endl;
}

void PCL::segmentate(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double threshould) {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// セグメンテーションオブジェクト作成 
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;

	seg.setOptimizeCoefficients(true);
  
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshould);

	seg.setInputCloud(cloud.makeShared());
	seg.segment(*inliers, *coefficients);

	for (size_t i = 0; i < inliers->indices.size(); ++i) {
		cloud.points[inliers->indices[i]].r = 255;
		cloud.points[inliers->indices[i]].g = 0;
		cloud.points[inliers->indices[i]].b = 0;
	}
	cout << "segmentate" << endl;
}

void PCL::convert_xyz_xyzrgb(pcl::PointCloud<pcl::PointXYZ> cloud_xyz, pcl::PointCloud<pcl::PointXYZRGB> &cloud_xyzrgb){
	pcl::copyPointCloud(cloud_xyz, cloud_xyzrgb);
	/*cloud_xyzrgb.points.resize(cloud_xyz.size());
	for (size_t i = 0; i < cloud_xyz.points.size(); i++) {
	cloud_xyzrgb.points[i].x = cloud_xyz.points[i].x;
	cloud_xyzrgb.points[i].y = cloud_xyz.points[i].y;
	cloud_xyzrgb.points[i].z = cloud_xyz.points[i].z;
	}*/

	cout << "convert_xyz_xyzrgb" << endl;
}

void PCL::divide_surface(Point data, Normal Normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output){
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(5);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(200);
	reg.setInputCloud(data);
	//reg.setIndices (indices);
	reg.setInputNormals(Normals);
	//reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);

	reg.setCurvatureThreshold(2.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	output = reg.getColoredCloud();
	cout << "divide surface" << endl;
}

void PCL::divide_surface_normal(Point data, Normal Normals, pcl::PointCloud<pcl::PointXYZ>::Ptr &output){
	output->points.resize(data->points.size());
	int count = 0;
	for (int i = 0; i < Normals->points.size(); i++){
		if (/*abs(Normals->points[i].normal_y) < parameter.theta[1] 
			&& abs(Normals->points[i].normal_z) < parameter.theta[2] */
			abs(Normals->points[i].normal_x)>parameter.theta[0]
			&& data->points[i].x > 0){
			output->points[count].x = data->points[i].x;
			output->points[count].y = data->points[i].y;
			output->points[count].z = data->points[i].z;
			count++;
		}
	}
	cout << "point : " << count << endl;
}


void PCL::correspond_spinimages(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::Histogram<153> >::Ptr model_descriptors, pcl::PointCloud<pcl::Histogram<153> >::Ptr scene_descriptors, pcl::CorrespondencesPtr &model_scene_corrs){
	pcl::KdTreeFLANN<pcl::Histogram<153>> match_search;
	
	match_search.setInputCloud(model_descriptors);
	/*std::vector<int> model_good_keypoints_indices;
	std::vector<int> scene_good_keypoints_indices;*/
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).histogram[0]))  
		{
			continue;
		}
		
		
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
			/*model_good_keypoints_indices.push_back(corr.index_query);
			scene_good_keypoints_indices.push_back(corr.index_match);*/
		}
	}
	
	/*Point model_good_kp(new pcl::PointCloud<pcl::PointXYZ>);
	Point scene_good_kp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
	pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);*/

	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
}

void PCL::correspond_shot(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors, pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors, pcl::CorrespondencesPtr model_scene_corrs){
	pcl::KdTreeFLANN<pcl::SHOT352> match_search;

	match_search.setInputCloud(model_descriptors);
	std::vector<int> neigh_indices(1);
	std::vector<float> neigh_sqr_dists(1);
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0]))
		{
			continue;
		}

		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

		if (found_neighs == 1 && neigh_sqr_dists[0] > 0.08f)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
			
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
}

void PCL::correspond_curvature(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr model_curvature, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr scene_curvature, pcl::CorrespondencesPtr model_scene_corrs){
	pcl::KdTreeFLANN<pcl::PrincipalCurvatures> match_search;
	match_search.setInputCloud(model_curvature);
	std::vector<int> neigh_indices(1);
	std::vector<float> neigh_sqr_dists(1);
	/*std::vector<int> model_good_keypoints_indices;
	std::vector<int> scene_good_keypoints_indices;*/
	for (size_t i = 0; i < scene_curvature->size(); ++i)
	{
		if (!pcl_isfinite(scene_curvature->at(i).pc1))
		{
			continue;
		}

		//if (scene_curvature->at(i).pc1 - scene_curvature->at(i).pc2 < 0.05){ //bunny  
		//if (scene_curvature->at(i).pc1 - scene_curvature->at(i).pc2 > 0.03){ //第二の船
		//	continue;
		//}

		int found_neighs = match_search.nearestKSearch(scene_curvature->at(i), 1, neigh_indices, neigh_sqr_dists);
		//if (found_neighs == 1 && neigh_sqr_dists[0] < 0.001f) //bunny model
		//if (found_neighs == 1 && neigh_sqr_dists[0] < 0.3f) //第二の船
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.01f) //第二の船(assemble)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
			
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
}

void PCL::correspond_original(Point model_keypoints, Point scene_keypoints, pcl::PointCloud<pcl::Histogram<num_hierarchy * 2>>::Ptr model_curvature, pcl::PointCloud<pcl::Histogram<num_hierarchy * 2>>::Ptr scene_curvature, pcl::CorrespondencesPtr model_scene_corrs){

	pcl::KdTreeFLANN<pcl::Histogram<num_hierarchy * 2>> match_search;
	//pcl::KdTreeFLANN<PC> match_search;


	match_search.setInputCloud(model_curvature);

	std::vector<int> neigh_indices(1);
	std::vector<float> neigh_sqr_dists(1);

	for (size_t i = 0; i < scene_curvature->points.size(); ++i)
	{
		//cout << i <<"/"<<scene_curvature->points.size()<< endl;
		if (!pcl_isfinite(scene_curvature->points[i].histogram[0]))
		{
			continue;
		}

		//if (scene_curvature->points[i].histogram[0] - scene_curvature->points[i].histogram[1] > 0.05){ //第二の船
		//	continue;
		//}

		int found_neighs = match_search.nearestKSearch(scene_curvature->points[i], 1, neigh_indices, neigh_sqr_dists);

		
		//if (found_neighs == 1 /*&& neigh_sqr_dists[0] < 0.0001f*/) //bunny model
		if (found_neighs == 1 /*&& neigh_sqr_dists[0] < 0.003*/) //第二の船
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			//corr.weight = 1.0 / neigh_sqr_dists[0];
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
}

void PCL::remove_incorrect(Point model, Point scene, pcl::CorrespondencesPtr model_scene_corrs, pcl::CorrespondencesPtr refine_corrs){
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> refine;
	refine.setInlierThreshold(0.0015);
	refine.setInputSource(model);
	refine.setInputTarget(scene);
	refine.setInputCorrespondences(model_scene_corrs);
	refine.getCorrespondences(*refine_corrs);
	cout << "remove incorrect matching " << endl;
}


void PCL::clustering(Point model, Point model_keypoints, Normal model_normals, Point scene, Point scene_keypoints, Normal scene_normals, pcl::CorrespondencesPtr model_scene_corrs, vector < pcl::Correspondences> &clustere_corrs, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans){
	bool use_hough_(false);
	float rf_rad_(0.15f);
	float cg_size_(0.05f);
	float cg_thresh_(5.0f);

	if (use_hough_)
	{
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
		pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

		pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);
		

		rf_est.setInputCloud(model_keypoints);
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model);
		rf_est.compute(*model_rf);

		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene);
		rf_est.compute(*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);
		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);
		//cout << model_scene_corrs->size() << endl;
		clusterer.recognize(rototrans, clustere_corrs);
		cout << "rototrans : "<<rototrans.size()<< endl;
		cout << "cludering Hough3D" << endl;
	}
	else
	{
		pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
		//cout << "size : " << corrs->size() << endl;

		gc_clusterer.recognize(rototrans, clustere_corrs);
		cout << "rototrans : " << rototrans.size() << endl;
		cout << "clustering" << endl;
	}
}

void PCL::Harris3D_SpinImage(char *f_name,char *f_name2){
	//一つ目の物体
	read(f_name, cloud);
	normal_calculation(cloud, normals);
	keypoints_calculation_Harris(cloud, keypoints_Harris);
	filtering(keypoints_Harris, keypoints_Harris);
	normal_calculation(keypoints_Harris, normals_keypoints);
	feature_calculation_spinimage(keypoints_Harris, normals_keypoints, spin_images);

	//二つ目の物体
	read(f_name2, cloud2);
	normal_calculation(cloud2, normals2);
	keypoints_calculation_Harris(cloud2, keypoints_Harris2);
	filtering(keypoints_Harris2, keypoints_Harris2);
	normal_calculation(keypoints_Harris2, normals_keypoints2);
	feature_calculation_spinimage(keypoints_Harris2, normals_keypoints2, spin_images2);

	//一致とクラスタリング
	correspond_spinimages(keypoints_Harris, keypoints_Harris2, spin_images, spin_images2, corrs);
	//remove_incorrect(keypoints_Harris, keypoints_Harris2, corrs);
	clustering(cloud, keypoints_Harris, normals, cloud2, keypoints_Harris2, normals2, corrs, clustered_corrs, rototranslations);
	cout << "Harris3D and SpinImage" << endl;
}

void PCL::Harris3D_Shot(char *f_name, char *f_name2){
	//一つ目の物体
	read(f_name, cloud);
	normal_calculation(cloud, normals);
	keypoints_calculation_Harris(cloud, keypoints_Harris);
	//filtering(keypoints_Harris, keypoints_Harris);
	//normal_calculation(keypoints_Harris, normals_keypoints);
	feature_calculation_shot(cloud, normals, keypoints_Harris, shot,0);


	//二つ目の物体
	read(f_name2, cloud2);
	normal_calculation(cloud2, normals2);
	//reverse_normal(normals2,normals2);
	keypoints_calculation_Harris(cloud2, keypoints_Harris2);
	//filtering(keypoints_Harris2, keypoints_Harris2);
	//normal_calculation(keypoints_Harris2, normals_keypoints2);
	feature_calculation_shot(cloud2, normals2, keypoints_Harris2, shot2,0);

	//一致とクラスタリング
	correspond_shot(keypoints_Harris, keypoints_Harris2, shot, shot2, corrs);
	remove_incorrect(keypoints_Harris, keypoints_Harris2, corrs,corrs);
	clustering(cloud, keypoints_Harris, normals, cloud2, keypoints_Harris2, normals2, corrs, clustered_corrs, rototranslations);
	cout << "Harris3D and Shot" << endl;
}

void PCL::Harris3D_Curvature(char *f_name, char *f_name2){
	const float radius = 0.3;

	//一つ目の物体
	read(f_name, cloud);
	smoothing(cloud,cloud);
	normal_calculation(cloud, normals);
	//create_mesh(cloud, normals, cloud_normals, mesh);
	flip_normal(cloud,normals);
	keypoints_calculation_Harris(cloud, keypoints_Harris); 
    
	//filtering(keypoints_Harris, keypoints_Harris);
	normal_calculation(keypoints_Harris, normals_keypoints);
	flip_normal(keypoints_Harris, normals_keypoints);
	Principal_Curvature(keypoints_Harris,normals_keypoints,principal_curvatures,radius);

	//二つ目の物体
	read(f_name2, cloud2);
	//smoothing(cloud2, cloud2);
	normal_calculation(cloud2, normals2);
	flip_normal(cloud2, normals2);
	keypoints_calculation_Harris(cloud2, keypoints_Harris2);
	//filtering(keypoints_Harris2,keypoints_Harris2);
	normal_calculation(keypoints_Harris2, normals_keypoints2);
	flip_normal(keypoints_Harris2, normals_keypoints2);
	//reverse_normal(normals2, normals2);
    //reverse_normal(normals_keypoints2, normals_keypoints2);
	Principal_Curvature(keypoints_Harris2, normals_keypoints2, principal_curvatures2,radius);

	cout << "before : " << principal_curvatures->points[0].pc1 << endl;
	cout << "after : " << principal_curvatures2->points[0].pc1 << endl;

	/*for (int i = 0; i < principal_curvatures2->size(); i++){
		float tmp = principal_curvatures2->points[i].pc1;
		principal_curvatures2->points[i].pc1 = -1.0 * principal_curvatures2->points[i].pc2;
		principal_curvatures2->points[i].pc2 = -1.0 * tmp;
	}*/

	//一致とクラスタリング
	correspond_curvature(keypoints_Harris, keypoints_Harris2, principal_curvatures, principal_curvatures2, corrs);
	//remove_incorrect(keypoints_Harris, keypoints_Harris2, corrs);
	clustering(cloud, keypoints_Harris, normals, cloud2, keypoints_Harris2, normals2, corrs, clustered_corrs, rototranslations);
	//registration(cloud,cloud2, rototranslations, cloud_final);
	cout << "Harris3D and Curvature" << endl;
}

void PCL::Harris3D_Original(char *f_name, char *f_name2){
	const float radius_rate = 0.05;//半径の増加率
	const float initial = 0.05;//半径の初期値

	//一つ目の物体
	read(f_name, cloud);
	smoothing(cloud, cloud);
	normal_calculation(cloud, normals);
	//create_mesh(cloud, normals, cloud_normals, mesh);
	flip_normal(cloud, normals);
	keypoints_calculation_Harris(cloud, keypoints_Harris);
	//filtering(keypoints_Harris, keypoints_Harris);
	normal_calculation(keypoints_Harris, normals_keypoints);
	flip_normal(keypoints_Harris, normals_keypoints);
	Hierarchy_Principal_Curvature(keypoints_Harris, normals_keypoints, Hierarchy_PC, radius_rate, initial, num_hierarchy, 0);

	/*for (int i = 0; i < Hierarchy_PC->points.size(); i++){
		for (int j = 0; j < Hierarchy_PC->points[i].pc1.size(); j++){
			cout << i << "_ pc1 : " << Hierarchy_PC->points[i].pc1[j] << endl;
			cout << i << "_ pc2 : " << Hierarchy_PC->points[i].pc2[j] << endl;
		}
	}*/

	//二つ目の物体
	read(f_name2, cloud2);
	smoothing(cloud2, cloud2);
	normal_calculation(cloud2, normals2);
	flip_normal(cloud2, normals2);
	keypoints_calculation_Harris(cloud2, keypoints_Harris2);
	//filtering(keypoints_Harris2,keypoints_Harris2);
	normal_calculation(keypoints_Harris2, normals_keypoints2);
	flip_normal(keypoints_Harris2, normals_keypoints2);
	Hierarchy_Principal_Curvature(keypoints_Harris2, normals_keypoints2, Hierarchy_PC2, radius_rate, initial, num_hierarchy, 0);

	//cout << "before : " << principal_curvatures->points[0].pc2 << endl;
	//cout << "after : " << principal_curvatures2->points[0].pc2 << endl;

	/*for (int i = 0; i < principal_curvatures2->size(); i++){
	float tmp = principal_curvatures2->points[i].pc1;
	principal_curvatures2->points[i].pc1 = -1.0 * principal_curvatures2->points[i].pc2;
	principal_curvatures2->points[i].pc2 = -1.0 * tmp;
	}*/

	//一致とクラスタリング
	correspond_original(keypoints_Harris, keypoints_Harris2, Hierarchy_PC, Hierarchy_PC2, corrs);
	remove_incorrect(keypoints_Harris, keypoints_Harris2, corrs, corrs);
	clustering(cloud, keypoints_Harris, normals, cloud2, keypoints_Harris2, normals2, corrs, clustered_corrs, rototranslations);
	//registration(cloud,cloud2, rototranslations, cloud_final);
	cout << "Harris3D and Original" << endl;
}

int PCL::instance_rotate(Point input, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans ,Point output){
	
	if (rototrans.size() <= 0)
	{
		cout << "*** No instances found! ***" << endl;
		return (0);
	}
	else
	{
		cout << "Recognized Instances: " << rototrans.size() << endl << endl;
	}

	
	/*for (size_t i = 0; i < rototrans.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::transformPointCloud(*input, *rotated_model, rototrans[i]);
		output.push_back(rotated_model);
	}*/
	pcl::transformPointCloud(*input, *output, rototrans[0]);
}

void PCL::ICP(Point model,Point scene,Point output){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputCloud(model);
	icp.setInputTarget(scene);
	icp.align(*output);
	cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
	cout << icp.getFinalTransformation() << std::endl;
}


void PCL::registration(Point model,Point scene, vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &rototrans, Point output){
	Point moved;
	instance_rotate(model,rototrans, moved);	
	//ICP(moved,scene,output);
	cout << "registration" << endl;
}

void PCL::find_hole(Point input){
	double max_x = 0, min_x = 0;
	double max_y = 0,min_y = 0;
	double max_z = 0, min_z = 0;

	for (int i = 0; i < input->points.size(); i++){
		//x
		if (max_x < input->points[i].x){
			max_x = input->points[i].x;
		}
		if (min_x > input->points[i].x){
			min_x = input->points[i].x;
		}

		//y
		if (max_y < input->points[i].y){
			max_y = input->points[i].y;
		}
		if (min_y > input->points[i].y){
			min_y = input->points[i].y;
		}

		//z
		if (max_z < input->points[i].z){
			max_z = input->points[i].z;
		}
		if (min_z > input->points[i].z){
			min_z = input->points[i].z;
		}

		//密度
		vector<int> num;
		for (float i = min_y; i < max_y; i = i + 0.01){
			for (float j = min_z; j < max_z; j = j + 0.01){

			}
		}
	}
	cout << "max : " << max_y << endl;
	cout << "min : " << min_y << endl;

	
}


void PCL::visualize(){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	//色の設定
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 100, 100, 100);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_smooth_color_handler(cloud_smooth, 200, 200, 200);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_registrated_color_handler(cloud_final, 255, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(keypoints_Harris, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler2(keypoints_Harris2, 0, 255, 0);

	//表示するものを選択
	//viewer->addPolygonMesh(mesh, "meshes", 0);
	//viewer->addPointCloud(cloud, cloud_color_handler, "cloud");
	//viewer->addPointCloud(cloud2, cloud_color_handler, "cloud2");
	//viewer->addPointCloud(cloud_rgb, "cloud_divide");
	//viewer->addPointCloud(cloud_smooth, cloud_smooth_color_handler, "cloud_smooth");

	//viewer->addPointCloud(keypoints_iss, keypoints_color_handler, "keypoints");
	viewer->addPointCloud(keypoints_Harris, keypoints_color_handler, "keypoints");
	viewer->addPointCloud(keypoints_Harris2, keypoints_color_handler2, "keypoints2");

	//一致した点同士を線でつなぐ
	bool show_correspondences_(true);
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				pcl::PointXYZ& model_point = keypoints_Harris->at(clustered_corrs[i][j].index_query);
				pcl::PointXYZ& scene_point = keypoints_Harris2->at(clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 255, 0, 0, ss_line.str());
			}
		}
	}

	//レジストレーション結果
	//viewer->addPointCloud(cloud_final, cloud_registrated_color_handler, "cloud_final");

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void PCL::move_center(Point input, float x, float y, float z){
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(input);
	mean = pca.getMean();
	for (int i = 0; i < input->size(); i++){
		input->points[i].x = input->points[i].x - mean.x() + x;
		input->points[i].y = input->points[i].y - mean.y() + y;
		input->points[i].z = input->points[i].z - mean.z() + z;
	}
}

void PCL::initialize_axes(Point input){
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(input);
	eigen_vector = pca.getEigenVectors();
	eigen_value = pca.getEigenValues();

	Eigen::Matrix3f rotate_Matrix, angle;

	//オブジェクト座標軸とワールド座標軸の角度差を計算し一致
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			angle(i, j) = acos(eigen_vector(i,j));
		}
	}
	
	rotate_Matrix(0, 0) = cos(angle(1, 0));
	rotate_Matrix(0, 1) = cos(angle(1, 1));
	rotate_Matrix(0, 2) = cos(angle(1, 2));
	rotate_Matrix(1, 0) = cos(angle(0, 0));
	rotate_Matrix(1, 1) = cos(angle(0, 1));
	rotate_Matrix(1, 2) = cos(angle(0, 2));
	rotate_Matrix(2, 0) = cos(angle(2, 0));
	rotate_Matrix(2, 1) = cos(angle(2, 1));
	rotate_Matrix(2, 2) = cos(angle(2, 2));

	Eigen::Vector3f tmp;
	for (int i = 0; i < input->points.size(); i++){
		tmp[0] = rotate_Matrix(0, 0) * input->points[i].x + rotate_Matrix(0, 1) * input->points[i].y + rotate_Matrix(0, 2) * input->points[i].z;
		tmp[1] = rotate_Matrix(1, 0) * input->points[i].x + rotate_Matrix(1, 1) * input->points[i].y + rotate_Matrix(1, 2) * input->points[i].z;
		tmp[2] = rotate_Matrix(2, 0) * input->points[i].x + rotate_Matrix(2, 1) * input->points[i].y + rotate_Matrix(2, 2) * input->points[i].z;
		input->points[i].x = tmp.x();
		input->points[i].y = tmp.y();
		input->points[i].z = tmp.z();
	}
}


void PCL::processing_with_2D(char *f_name){
	read(f_name, cloud);
	move_center(cloud, 0, 0, 0);
	initialize_axes(cloud);
	//smoothing(cloud,cloud);
	normal_calculation(cloud, normals);
	flip_normal(cloud,normals);
	divide_surface_normal(cloud, normals, cloud2);
	//filtering(cloud, cloud);
	//find_hole(cloud2);
	keypoints_calculation_Harris(cloud2, keypoints_Harris);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);

	//メッシュ
	/*normal_calculation(cloud, normals);
	create_mesh(cloud, normals, cloud_normals, mesh);
	viewer->addPolygonMesh(mesh, "mesh", 0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, "mesh");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "mesh");*/
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "mesh");
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 255, "mesh");
	
	//ポイントクラウド
	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 100, 100, 100);
	viewer->addPointCloud(cloud, cloud_color_handler, "cloud");*/

	//ポイントクラウド(面を分けたもの)
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> surface_color_handler(cloud, 100, 0, 0);
	viewer->addPointCloud(cloud2, surface_color_handler, "cloud2");
	
	//キーポイント
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>keypoint_color_handler(keypoints_Harris, 0, 100, 0);
	viewer->addPointCloud(keypoints_Harris, keypoint_color_handler, "Harris");

	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(parameter.view_point[0], parameter.view_point[1], parameter.view_point[2], 0, 1, 0);
	viewer->setShowFPS(false);
	viewer->setSize(700, 700);
	
	while (!viewer->wasStopped()){
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
