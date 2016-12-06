#ifndef CORRESPONDING_POINT
#define CORRESPONDING_POINT

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#ifdef _DEBUG
#pragma comment(lib,"opencv_highgui249d.lib")
#pragma comment(lib,"opencv_core249d.lib")
#pragma comment(lib,"opencv_imgproc249d.lib")
#pragma comment(lib,"opencv_nonfree249d.lib")
#pragma comment(lib,"opencv_features2d249d.lib")
#pragma comment(lib,"opencv_legacy249d.lib")
#pragma comment(lib,"opencv_calib3d249d.lib")
#else
#pragma comment(lib,"opencv_highgui249.lib")
#pragma comment(lib,"opencv_core249.lib")
#pragma comment(lib,"opencv_imgproc249.lib")
#pragma comment(lib,"opencv_nonfree249.lib")
#pragma comment(lib,"opencv_features2d249.lib")
#pragma comment(lib,"opencv_legacy249.lib")
#pragma comment(lib,"opencv_calib3d249.lib")
#pragma comment(lib,"opencv_video249.lib")
#endif

using namespace std;
using namespace cv;

class Corresponding_point
{
public:
	Corresponding_point();
	~Corresponding_point();
	int image_view(string f_name1, string f_name2);
	void copy_img(Mat img1, Mat &img2, int x, int y);//img1をimg2にx,y平行移動させてコピー
	void calculate_hole_position(Mat img, vector<Point2d> &position);//穴の場所を探す
	void sort_hole_position(vector<Point2d> position, vector<Point2d> &output);//穴のポジションをソート
	void draw_line(Mat img, vector<Point2d> pos1, vector<Point2d> pos2);
	void Blending(Mat input1, Mat input2, Mat &output);
	void DP_Matching(vector<Point2d> pos1, vector<Point2d> pos2, vector<Point2d> &pair1, vector<Point2d> &pair2, double &Error);
	void calculate_R_T(vector<Point2d> pos1, vector<Point2d> pos2, Mat &R, Mat &T, Mat &D);//回転，平行移動行列の計算と合成
	void multiple_Mat_point2f(Point2d pos, Point2d &out_pos, Mat M);//行列とPoint2dの掛け算
	void R_T_point(vector<Point2d> pos, vector<Point2d> &out_pos, Mat D);//点に回転，平行移動を適応
	void Point2d_to_Mat(vector<Point2d> input, Mat &output);//形式の変換
	void calculate_centroid(Mat input, Mat &output);//中心の計算
private:
	vector<Point2d> hole_pos;
	vector<Point2d> hole_pos2;
	vector<Point2d> corresponding_pos1;
	vector<Point2d> corresponding_pos2;
	double error;
	Mat Rotation, Translation, M_deformation;
	vector<Mat> all_deformation;
};



#endif

