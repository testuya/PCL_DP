#include "Corresponding_point.h"
#include "parameter.h"

Parameter parameter;

Corresponding_point::Corresponding_point()
{
}

Corresponding_point::~Corresponding_point()
{
}

void Corresponding_point::copy_img(Mat img1, Mat &img2, int x, int y){
	for (int i = y; i < img1.rows + y; i++){
		for (int j = x; j < img1.cols + x; j++){
			for (int k = 0; k < img1.channels(); k++){
				img2.data[i * img2.step + j * img2.elemSize() + k] = img1.data[(i - y) * img1.step + (j - x) * img1.elemSize() + k];
			}
		}
	}
}

void Corresponding_point::calculate_hole_position(Mat img, vector<Point2d> &position){
	//赤い点の場所格納
	bool flag = false;//最初の格納
	int count = 0;//格納数
	position.resize(0);

	for (int i = 0; i < img.rows; i++){
		for (int j = 0; j < img.cols; j++){
			if (int(img.data[i * img.step + j * img.elemSize()]) != 255 &&
				int(img.data[i * img.step + j * img.elemSize() + 1]) != 255 &&
				int(img.data[i * img.step + j * img.elemSize() + 2]) == 255){

				//if (flag == false){
				//	flag = true;
				//	Point2d tmp(j, i);
				//	position.push_back(tmp);
				//	count++;
				//	//cout << j << ":" <<  i  << endl;

				//}
				//else{
				//	if (abs(j - position[count - 1].x) + abs(i - position[count - 1].y) > parameter.hole_point_size){
				//		Point2d tmp(j, i);
				//		position.push_back(tmp);
				//		count++;
				//		//cout << j << ":" <<  i  << endl;

				//	}
				//}
				if (position.size() == 0){
					Point2d tmp(j, i);
					position.push_back(tmp);
					count++;
					//cout << "count : " << count << endl;
				}
				else{
					for (int k = 0; k < position.size(); k++){
						if (abs(j - position[k].x) + abs(i - position[k].y) < parameter.hole_point_size){
							//if (sqrt((j - position[k].x) *(j - position[k].x) + (i - position[k].y) * (i - position[k].y)) < parameter.hole_point_size){
							flag = true;
						}
					}

					if (flag != true){
						Point2d tmp(j, i);
						position.push_back(tmp);
						count++;
						//cout << "count : " << count << endl;

					}
					flag = false;
				}
			}
		}
	}
}

void Corresponding_point::sort_hole_position(vector<Point2d> position, vector<Point2d> &output){
	//重心の計算
	Point2d centroid(0, 0);
	for (int i = 0; i < position.size(); i++){
		centroid += position[i];
	}
	centroid.x = centroid.x / position.size();
	centroid.y = centroid.y / position.size();

	//重心から近い順番にソート
	double max_distance;
	int id = 0;
	output.resize(0);
	while (output.size() != position.size()){
		max_distance = 0;
		for (int i = 0; i < position.size(); i++){
			double d = (position[i].x - centroid.x)*(position[i].x - centroid.x) + (position[i].y - centroid.y)*(position[i].y - centroid.y);
			if (max_distance < d){
				max_distance = d;
				id = i;
			}
		}
		output.push_back(position[id]);
		position[id].x = centroid.x;
		position[id].y = centroid.y;
	}

}


void Corresponding_point::draw_line(Mat img, vector<Point2d> pos1, vector<Point2d> pos2){
	Point2d size(700, 0);
	int min_size;
	if (pos1.size() < pos2.size()){
		min_size = pos1.size();
	}
	else{
		min_size = pos2.size();
	}

	//cout << min_size;

	for (int i = 0; i < min_size; i++){
		line(img, pos1[i], size + pos2[i], Scalar(0, 0, 255));

	}
	//imshow("test", img);
}

void Corresponding_point::Blending(Mat input1, Mat input2, Mat &output){
	Mat result = input1.clone();
	for (int i = 0; i < input1.rows; i++){
		for (int j = 0; j < input1.cols; j++){
			for (int k = 0; k < input1.channels(); k++){
				if ((int)result.data[i * input1.step + j * input1.elemSize() + k] == 255){
					result.data[i * input1.step + j * input1.elemSize() + k] = input2.data[i * input2.step + j*input2.elemSize() + k];
				}
				//cout << src.data[y * src.step + x * src.elemSize() + c] << endl;
			}

		}
	}

	output = result.clone();
}

void Corresponding_point::DP_Matching(vector<Point2d> pos1, vector<Point2d> pos2, vector<Point2d> &pair1, vector<Point2d> &pair2, double &Error){
	const int height = pos1.size() - 1;
	const int width = pos2.size() - 1;

	double **error = new double *[height];//エラー値の格納 
	double **cost = new double *[height];//コストの格納
	int **route = new int *[height];//ルートの格納(斜め：0 縦：1  横：2  )
	double *distance1 = new double[height];//1つ目の特徴量間の距離
	double *distance2 = new double[width];//2つ目の特徴量間の距離
	pair1.resize(0);
	pair2.resize(0);

	//動的確保
	for (int i = 0; i < height; i++){
		error[i] = new double[width];
		cost[i] = new double[width];
		route[i] = new int[width];
	}

	//distanceを計算
	for (int i = 0; i < height; i++){
		int count = 0;
		distance1[i] = sqrt((pos1[i].x - pos1[i + 1].x) * (pos1[i].x - pos1[i + 1].x) + (pos1[i].y - pos1[i + 1].y) * (pos1[i].y - pos1[i + 1].y));
		cout << "1 :" << i << " : " << distance1[i] << endl;

	}

	for (int i = 0; i < width; i++){
		int count = 0;

		distance2[i] = sqrt((pos2[i].x - pos2[i + 1].x) * (pos2[i].x - pos2[i + 1].x) + (pos2[i].y - pos2[i + 1].y) * (pos2[i].y - pos2[i + 1].y));
		cout << "2 :" << i << " :" << distance2[i] << endl;
		count++;
	}
	cout << endl;

	//errorを総当たりで計算
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			if (abs(distance1[i] - distance2[j]) < parameter.Threshold_error){
				error[i][j] = 0;
			}
			else{
				error[i][j] = 1;
			}
			cout << error[i][j] << "   ";
		}
		cout << endl;
	}

	/*コストとルートの計算*/
	//原点の計算
	cost[0][0] = error[0][0];
	route[0][0] = 1;

	//0列目の計算
	for (int j = 1; j < height; j++) {
		cost[j][0] = cost[j - 1][0] + error[j][0] * parameter.DPM_different + parameter.DPM_displacement;
		route[j][0] = 1;
		//cout << "Route : " << j  << endl;
	}

	//0行目の計算
	for (int i = 1; i < width; i++) {
		cost[0][i] = cost[0][i - 1] + error[0][i] * parameter.DPM_different + parameter.DPM_displacement;
		route[0][i] = 2;
		//cout << "Route : " << i << endl;
	}


	//その他
	int tmp[3];
	for (int i = 1; i < height; i++) {
		for (int j = 1; j < width; j++) {
			tmp[0] = cost[i - 1][j - 1] + error[i][j] * parameter.DPM_different * 2; //斜めで来た場合のコスト 
			tmp[1] = cost[i - 1][j] + error[i][j] * parameter.DPM_different + parameter.DPM_displacement; //i増えで来た場合のコスト 
			tmp[2] = cost[i][j - 1] + error[i][j] * parameter.DPM_different + parameter.DPM_displacement; //j増えで来た場合のコスト 

			//最小コストの格納
			if (tmp[0] <= tmp[1] && tmp[0] <= tmp[2]) {
				cost[i][j] = tmp[0];
				route[i][j] = 0;
			}
			else if (tmp[1] < tmp[2]) {
				cost[i][j] = tmp[1];
				route[i][j] = 1;
			}
			else {
				cost[i][j] = tmp[2];
				route[i][j] = 2;
			}
			//cout <<"Route : " << i * width + j <<  endl;
		}
	}

	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			cout << "cost " << i << " : " << j << " = " << cost[i][j] << "    ";
		}
		cout << endl;
	}

	cout << endl;

	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			cout << "route " << i << " : " << j << " = " << route[i][j] << "    ";
		}
		cout << endl;
	}

	Error = cost[height - 1][width - 1];
	cout << "error_distance : " << Error << endl;

	//最小のコストルートの検索
	int Len = width + height;
	int row = height - 1;
	int col = width - 1;
	int *result1 = new int[Len + 1];
	int *result2 = new int[Len + 1];

	for (int i = 0; i < Len; i++){
		result1[i] = 0;
		result2[i] = 0;
	}

	int l;
	//for (int i = 0; row >= 0 && col >= 0; i++) {
	for (l = Len; row >= 0 && col >= 0; l--) {
		/*result1[l] = row;
		result2[l] = col;*/
		/*result1[i] = row;
		result2[i] = col; */

		switch (route[row][col]) {
		case 0:
			row--;
			col--;
			break;
		case 1:
			row--;
			break;
		case 2:
			col--;
			break;
		default:
			cout << "Route Error : " << route[row][col] << endl;;
			break;
		}

	}

	//Len -= l; 

	/*for (int i = 0; i < Len; i++) {
	result1[i] = result1[i + l + 1];
	result2[i] = result2[i + l + 1];
	}*/

	//結果の表示
	/*for (int i = 0; i < Len ; i++){
	cout <<"result1 : " << result1[i] << endl;
	cout <<"result2 : " << result2[i] << endl;
	}*/

	int i = height - 1;
	int j = width - 1;
	int k;
	for (k = Len; i >= 0 && j >= 0; k--) {
		result1[k] = i;
		result2[k] = j;

		//printf("%c %c  ", ResultA[k], ResultB[k]); 
		//cout << route[i][j] << endl;
		switch (route[i][j]) {
		case 0:
			/*cout << i << " : " << j << endl;
			pair1.push_back(pos1[i]);
			pair2.push_back(pos2[j]);*/
			i--;
			j--;
			if (cost[i + 1][j + 1] - cost[i][j] == 0){
				cout << i + 1 << " : " << j + 1 << " : " << error[i + 1][j + 1] << endl;
				pair1.push_back(pos1[i + 1]);
				pair2.push_back(pos2[j + 1]);
				pair1.push_back(pos1[i]);
				pair2.push_back(pos2[j]);
			}
			break;
		case 1:
			i--;
			break;
		case 2:
			j--;
			break;
		default:
			printf("Error\n");
			break;
		}
	}

	delete[] error;
	delete[] cost;
	delete[] route;
	delete[] result1;
	delete[] result2;
}

void Corresponding_point::multiple_Mat_point2f(Point2d pos, Point2d &out_pos, Mat M){

	out_pos.x = pos.x * M.at<double>(0, 0) + pos.y * M.at<double>(1, 0) + M.at<double>(2, 0);
	out_pos.y = pos.x * M.at<double>(0, 1) + pos.y * M.at<double>(1, 1) + M.at<double>(2, 1);
}


void Corresponding_point::calculate_R_T(vector<Point2d> pos1, vector<Point2d> pos2, Mat &R, Mat &T, Mat &D){
	//型の変換
	Mat M_pos1, M_pos2;
	Point2d_to_Mat(pos1, M_pos1);
	Point2d_to_Mat(pos2, M_pos2);

	//中心の計算
	Mat centroid1, centroid2;
	calculate_centroid(M_pos1, centroid1);
	calculate_centroid(M_pos2, centroid2);

	//中心を引く
	M_pos1.row(0) -= centroid1.at<double>(0, 0);
	M_pos1.row(1) -= centroid1.at<double>(1, 0);
	M_pos2.row(0) -= centroid2.at<double>(0, 0);
	M_pos2.row(1) -= centroid2.at<double>(1, 0);
	cout << centroid1 << endl;
	cout << centroid2 << endl;

	Mat H = M_pos1 * M_pos2.t();
	Mat W, U, Vt;
	SVDecomp(H, W, U, Vt, SVD::FULL_UV);

	R = U * Vt;
	T = centroid2.t() - centroid1.t() * R;
	D = Mat_<double>(3, 2);
	D.at<double>(0, 0) = R.at<double>(0, 0);
	D.at<double>(1, 0) = R.at<double>(1, 0);
	D.at<double>(2, 0) = T.at<double>(0, 0);
	D.at<double>(0, 1) = R.at<double>(0, 1);
	D.at<double>(1, 1) = R.at<double>(1, 1);
	D.at<double>(2, 1) = T.at<double>(0, 1);

}

void Corresponding_point::R_T_point(vector<Point2d> pos, vector<Point2d> &out_pos, Mat D){
	out_pos.resize(0);

	for (int i = 0; i < pos.size(); i++){
		Point2d tmp;
		multiple_Mat_point2f(pos[i], tmp, D);
		out_pos.push_back(tmp);

	}
}

void Corresponding_point::Point2d_to_Mat(vector<Point2d> input, Mat &output){
	Mat tmp = (Mat_<double>(2, input.size()));

	for (int i = 0; i < input.size(); i++){
		tmp.at<double>(0, i) = input[i].x;
		tmp.at<double>(1, i) = input[i].y;
	}
	output = tmp.clone();
}

void Corresponding_point::calculate_centroid(Mat input, Mat &output){
	//初期化
	Mat tmp = (Mat_<double>(2, 1));
	tmp.at<double>(0, 0) = input.at<double>(0, 0);
	tmp.at<double>(1, 0) = input.at<double>(1, 0);

	//計算
	for (int i = 1; i < input.cols; i++){
		tmp.at<double>(0, 0) += input.at<double>(0, i);
		tmp.at<double>(1, 0) += input.at<double>(1, i);
	}
	tmp /= input.cols;
	output = tmp.clone();
}



int Corresponding_point::image_view(string f_name1, string f_name2){
	Mat src_img1 = imread(f_name1, 1);//入力画像1
	Mat src_img2 = imread(f_name2, 1);//入力画像2

	/*エラー処理*/
	if (!src_img1.data){
		cout << "cannot read : 1" << endl;
		return -1;
	};
	if (!src_img2.data){
		cout << "cannot read : 2" << endl;
		return -1;
	}

	calculate_hole_position(src_img1, hole_pos);
	calculate_hole_position(src_img2, hole_pos2);
	//sort_hole_position(hole_pos, hole_pos);
	//sort_hole_position(hole_pos2, hole_pos2);
	DP_Matching(hole_pos, hole_pos2, corresponding_pos1, corresponding_pos2, error);

	if (corresponding_pos1.size() > 4){
		Mat masks;
		Mat H = findHomography(corresponding_pos1, corresponding_pos2, masks, CV_RANSAC, parameter.ransac);
		vector<Point2d> ransac_corresponding_pos1;
		vector<Point2d> ransac_corresponding_pos2;
		for (size_t i = 0; i < masks.rows; ++i) {
			uchar *inliner = masks.ptr<uchar>(i);
			if ((int)inliner[0] == 1) {
				ransac_corresponding_pos1.push_back(corresponding_pos1[i]);
				ransac_corresponding_pos2.push_back(corresponding_pos2[i]);
			}
			//cout << (int)inliner[0] << endl;
		}

		calculate_R_T(ransac_corresponding_pos1, ransac_corresponding_pos2, Rotation, Translation, M_deformation);
		//calculate_R_T(corresponding_pos1, corresponding_pos2, Rotation, Translation, M_deformation);
		/*R_T_point(corresponding_pos1, corresponding_pos1, M_deformation);
		hole_pos = corresponding_pos1;
		hole_pos2 = corresponding_pos2;*/

		all_deformation.push_back(M_deformation);
	}
	else{
		calculate_R_T(corresponding_pos1, corresponding_pos2, Rotation, Translation, M_deformation);
		all_deformation.push_back(M_deformation);
	}

	Mat Affine_result, Blend_result;

	//計算した変形から画像生成
	Mat final_deformation = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	for (int i = 0; i < all_deformation.size(); i++){
		Mat tmp = (Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
		tmp.at<double>(0, 0) = all_deformation[i].at<double>(0, 0);
		tmp.at<double>(1, 0) = all_deformation[i].at<double>(1, 0);
		tmp.at<double>(2, 0) = all_deformation[i].at<double>(2, 0);
		tmp.at<double>(0, 1) = all_deformation[i].at<double>(0, 1);
		tmp.at<double>(1, 1) = all_deformation[i].at<double>(1, 1);
		tmp.at<double>(2, 1) = all_deformation[i].at<double>(2, 1);

		final_deformation = final_deformation * tmp;
	}
	Mat Correspond_result = Mat(src_img1.rows, src_img1.cols * 2, src_img1.type(), Scalar(255, 255, 255));
	copy_img(src_img1, Correspond_result, 0, 0);
	copy_img(src_img2, Correspond_result, src_img1.size().width, 0);
	draw_line(Correspond_result, corresponding_pos1, corresponding_pos2);

	imshow("Correspond_result", Correspond_result);
	cout << final_deformation(Rect(0, 0, 2, 3)).t() << endl;
	warpAffine(src_img1, Affine_result, final_deformation(Rect(0, 0, 2, 3)).t(), src_img1.size(), CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
	src_img1 = Affine_result.clone();

	Blending(src_img1, src_img2, Blend_result);
	imshow("Blend_result", Blend_result);
	//imshow("Affine_result", Affine_result);
	waitKey(0);
}
