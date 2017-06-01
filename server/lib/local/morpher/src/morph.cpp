#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <json.hpp>
#include <base64.h>

using json = nlohmann::json;
using namespace cv;
using namespace std;

/**
 * Extract points from the given text file
 *  and return a point-list in vector<Point2f> 
 *
 * @param filename 
 * @return vector<Point2f>
 */
vector<cv::Point2f> get_points_from_file(string filename){
    vector<Point2f> coords;
    char * stri= (char*) filename.c_str();
    ifstream inp(stri);
    float x=0, y=0;
    while(inp >> x){ // read points in pairs
        inp >> y;
        coords.push_back(cv::Point2f(x,y));
    }
    return coords;
}

/**
 * Apply affine transform calculated using srcTri and dstTri to src
 *
 * @param tri1
 * @param tri2
 * @param src (ref)
 * @param output (ref)
 * @return void
 */
void transform_triangle(vector<cv::Point2f> &tri1, vector<cv::Point2f> &tri2, cv::Mat &src, cv::Mat &output){

	// get the affine transform using function for the triangles
    cv::Mat transMat = cv::getAffineTransform(tri1, tri2);
    
	// transform the image based on transformation matrix obtained
    warpAffine( src, output, transMat, output.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT_101);
}

/**
 * Get a rectangle fully containing the triangle
 * @param img
 * @param Tri (ref)
 * @return cv::Mat
 */
cv::Mat getRectForTriangles(cv::Mat img, vector<Point2f> &Tri){

    int mx = img.cols; 
	int my = img.rows; 
	int Mx = 0;
	int My = 0;

    for (int i=0; i<Tri.size(); i++){
		int _x = cvRound(Tri[i].x);
		int _y = cvRound(Tri[i].y);
		mx = min(mx, _x); Mx = max(Mx, _x);
		my = min(my, _y); My = max(My, _y);
	}

    Point pp((Mx-mx)+1, (My-my)+1);
    Rect rect(mx, my, pp.x, pp.y);
    cv::Mat rec = img(rect);

	//printf("%d, %d, %d, %d\n", mx, my, pp.x, pp.y);
    return rec;
}


/** 
 * Point wise multiply matrices img and mask. 
 *  Removes all points which do not fall under the mask 
 *  by multiplication with 0
 *
 * @param img
 * @param mast (ref)
 * @return void
 */
void point_mult(cv::Mat &img, cv::Mat &mask){
	for (int yy=0; yy<img.rows; yy++){
		for (int xx=0; xx<img.cols; xx++){
			img.at<Vec3f>(yy, xx)[0] = img.at<Vec3f>(yy, xx)[0] * mask.at<Vec3f>(yy, xx)[0];
			img.at<Vec3f>(yy, xx)[1] = img.at<Vec3f>(yy, xx)[1] * mask.at<Vec3f>(yy, xx)[0];
			img.at<Vec3f>(yy, xx)[2] = img.at<Vec3f>(yy, xx)[2] * mask.at<Vec3f>(yy, xx)[0];
        }
    }
}

/**
 * Point wise multiply matrices 
 *  but with an offset for index of mask
 *
 * @param img
 * @param mask (ref)
 * @param rr
 * @param cc
 * @return void
 */
void point_mult2(cv::Mat &img, cv::Mat &mask, int rr, int cc){
	for(int yy=rr; yy<rr+mask.rows; yy++){
        for(int xx=cc; xx<cc+mask.cols; xx++){
			img.at<Vec3f>(yy, xx)[0] = img.at<Vec3f>(yy,xx)[0] * mask.at<Vec3f>(yy-rr, xx-cc)[0];
			img.at<Vec3f>(yy, xx)[1] = img.at<Vec3f>(yy,xx)[1] * mask.at<Vec3f>(yy-rr, xx-cc)[0];
			img.at<Vec3f>(yy, xx)[2] = img.at<Vec3f>(yy,xx)[2] * mask.at<Vec3f>(yy-rr, xx-cc)[0];
        }
    }
}

/**
 * Compute the intermediate transformed triangle 
 *  for both images and combine it based on imageRatio
 *
 * @param tri_src (ref)
 * @param tri_dst (ref)
 * @param tri_out (ref)
 * @param img1 (ref)
 * @param img2 (ref)
 * @param img (ref)
 * @param imageRatio
 * @return void
 */
void tri_morphing(vector<Point2f> &tri_src, vector<Point2f> &tri_dst, vector<Point2f> &tri_out, cv::Mat &img1, cv::Mat &img2, cv::Mat &img, double imageRatio){

    float mx = img1.cols; 
	float my = img1.rows; 
	float Mx = 0;
	float My = 0;

    for(int i=0; i<tri_src.size(); i++){
		mx = min(mx, tri_src[i].x); my = min(my, tri_src[i].y);
		Mx = max(Mx, tri_src[i].x); My = max(My, tri_src[i].y);
    }

	Point2f sp(mx, my);
    mx = img1.cols; 
	my = img1.rows; 
	Mx = My = 0;

    for(int i=0; i<tri_dst.size(); i++){
      mx = min(mx, tri_dst[i].x); my = min(my, tri_dst[i].y);
      Mx = max(Mx, tri_dst[i].x); My = max(My, tri_dst[i].y);
    }

    Point2f dp(mx, my);
    mx = img1.cols;
	my = img1.rows;
	Mx = My = 0;

    for(int i=0; i<tri_out.size(); i++){
		mx = min(mx, tri_out[i].x); my = min(my, tri_out[i].y);
		Mx = max(Mx, tri_out[i].x); My = max(My, tri_out[i].y);
    }

    Point2f ip(mx, my);

    cv::Mat rct_src = getRectForTriangles(img1, tri_src);
    if (rct_src.rows==0 || rct_src.cols==0 ){ return; }
	
    cv::Mat rct_dst = getRectForTriangles(img2, tri_dst);
    if(rct_dst.rows==0 || rct_dst.cols==0){ return; }

    cv::Mat rct_out = getRectForTriangles(img, tri_out);
    if(rct_out.rows==0 || rct_out.cols==0 ){ return; }

	vector<Point> mask_tri;
    for(int i = 0; i<3; i++){
		tri_out[i].x = (tri_out[i].x - ip.x); tri_out[i].y = (tri_out[i].y - ip.y);
		tri_src[i].x = (tri_src[i].x - sp.x); tri_src[i].y = (tri_src[i].y - sp.y);
		tri_dst[i].x = (tri_dst[i].x - dp.x); tri_dst[i].y = (tri_dst[i].y - dp.y);
		mask_tri.push_back(Point(tri_out[i].x, tri_out[i].y));
    }

    // create a mask
    cv::Mat resultMask = cv::Mat::zeros(rct_out.rows, rct_out.cols, CV_32FC3);

    // fill the triangle in mask with 1.0 value, rest 0
    fillConvexPoly(resultMask, mask_tri, cv::Scalar(1.0, 1.0, 1.0), CV_AA, 0);

    // get warped matrices
    cv::Mat sWarped = cv::Mat::zeros(rct_out.rows, rct_out.cols, rct_src.type());
    cv::Mat dWarped = cv::Mat::zeros(rct_out.rows, rct_out.cols, rct_dst.type());

    transform_triangle(tri_src, tri_out, rct_src, sWarped);
    transform_triangle(tri_dst, tri_out, rct_dst, dWarped);

    // combine the warped matrices by blending them based on ratio
    cv::Mat imgRect = imageRatio*dWarped + (1.0 - imageRatio)*sWarped ;

    point_mult(imgRect, resultMask);

    resultMask = cv::Scalar(1.0, 1.0, 1.0) - resultMask;
    Rect resRec = Rect(ip.x, ip.y, rct_out.cols, rct_out.rows);

    point_mult2(img, resultMask, ip.y, ip.x);
    img(resRec) = img(resRec) + imgRect;
}

bool contain_tri_in_rect(Rect r, Vec6f t){
	vector<Point> pt(3);
	pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
	pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
	pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
	//cout << "r:" << r.x << ", " << r.y << " (" << r.width << " x " << r.height << ")" << endl;

	if(r.contains(pt[0]) && r.contains(pt[1]) && r.contains(pt[2])){
		return true;
	}
	return false;
}

static void write_delaunay(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color, std::string filename){

    vector<Vec6f> triangle_list;
    subdiv.getTriangleList(triangle_list);
    vector<Point> pt(3);
    Rect rect(0, 0, img.cols, img.rows);

    for(size_t i=0; i<triangle_list.size(); i++){
        Vec6f t = triangle_list[i];
         
        // Draw rectangles completely inside the image.
        if(contain_tri_in_rect(rect, t)){
			pt[0] = Point(cvRound(t[0]), cvRound(t[1]));
			pt[1] = Point(cvRound(t[2]), cvRound(t[3]));
			pt[2] = Point(cvRound(t[4]), cvRound(t[5]));
            line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
            line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
            line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
        }
    }
	imwrite(filename.c_str(), img);
}

void parse_points_from_landmarks(json landmarks, vector<cv::Point2f> &points){
	for(json::iterator itr = landmarks.begin(); itr != landmarks.end(); ++itr) {
		points.push_back(cv::Point2f((*itr).at("x"), (*itr).at("y")));
	}
}

void points_to_subdiv(vector<cv::Point2f> points, cv::Subdiv2D &sdv){
	for(vector<Point2f>::iterator it = points.begin(); it != points.end(); it++){
		sdv.insert(*it);
	}
}

int get_index_from_points_vector(vector<cv::Point2f> points, cv::Point2f p){
	for(int i=0; i<points.size(); i++){
		if(p.x == points[i].x && p.y == points[i].y){
			return i;
		}
	}
	return -1;
}

void write_triangle_indices_to_file(cv::Subdiv2D &sdv, vector<cv::Point2f> points){

	vector<Vec6f> tri;
	sdv.getTriangleList(tri);
	
	vector<Point2f> pt(3);

	int idx1, idx2, idx3;

	ofstream ofs("tri-list.txt");

	for(int i=0; i<tri.size(); i++){
		
		Vec6f t = tri[i];
		pt[0] = Point2f(t[0], t[1]);
		pt[1] = Point2f(t[2], t[3]);
		pt[2] = Point2f(t[4], t[5]);

		idx1 = get_index_from_points_vector(points, pt[0]);
		idx2 = get_index_from_points_vector(points, pt[1]);
		idx3 = get_index_from_points_vector(points, pt[2]);

		if(idx1<0 || idx2<0 || idx3<0){ continue; }

		ofs << idx1 << " " << idx2 << " " << idx3 << endl;
	}
	ofs.close();
}

void get_json_from_file(const char *filename, json &j){
	ifstream ifs(filename);
	ifs >> j;
	ifs.close();
}

void encode_image_base64(cv::Mat mat, std::string &encoded){
	vector<uchar> buf;
	cv::imencode(".jpg", mat, buf);
	uchar *enc_msg = new uchar[buf.size()];
	for(int i=0; i<buf.size(); i++){
		enc_msg[i] = buf[i];
	}
	encoded = base64_encode(enc_msg, buf.size());
}

void crop_valid_area_from_image(cv::Mat &img_in, vector<cv::Point2f> &pts_out, cv::Mat &img_out){

	float min_x = img_in.cols-1; float max_x = 0;
	float min_y = img_in.rows-1; float max_y = 0;	
	for(int i=0; i<pts_out.size(); i++){
		cv::Point2f p = pts_out[i];
		min_x = min(min_x, p.x); 
		min_y = min(min_y, p.y);
		max_x = max(max_x, p.x); 
		max_y = max(max_y, p.y);
	}
	int width = static_cast<int>((max_x - min_x)*1.1);
	int height = static_cast<int>((max_y - min_y)*1.1);

	cv::Rect rect(static_cast<int>(min_x*0.9), static_cast<int>(min_y*0.9), width, height);
	img_out = img_in(rect);
}

void generate_random_prefix(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}

void add_8_more_points(cv::Mat &img, vector<cv::Point2f> &pts){
	pts.push_back(Point2f(1, 1));
	pts.push_back(Point2f(img.cols-2, 1));
	pts.push_back(Point2f((img.cols-2)/2, 1));
	pts.push_back(Point2f(1, img.rows-2));
	pts.push_back(Point2f(1, (img.rows-2)/2));
	pts.push_back(Point2f(img.cols-2, (img.rows-2)/2));
	pts.push_back(Point2f((img.cols-2)/2, img.rows-2));
	pts.push_back(Point2f(img.cols-2, img.rows-2));
}


/**
 * perform face morphing
 *
 * @param img_src
 * @param img_dst
 * @param jsn_fld
 * @param target_result_idx
 * @param encoded_src
 * @param encoded_dst
 * @param encoded_out
 */
bool face_morph(cv::Mat &img_src, cv::Mat &img_dst, json &jsn_fld, int target_result_idx, string &encoded_src, string &encoded_dst, string &encoded_out){

	/** constants */
	double NUM_ITERATIONS = 10;
	Scalar COLOR_WHITE(255, 255, 255);

	const int SIZE_TRI_LIST = 142;
	int tri_list[SIZE_TRI_LIST][3] = {{40,37,38}, {37,40,41}, {2,72,1}, {72,2,3}, {0,72,68}, {72,0,1}, {31,2,1}, {2,31,48}, {1,0,36}, {31,1,41}, {72,3,4}, {1,36,41}, {71,4,5}, {4,71,72}, {3,2,48}, {4,48,5}, {48,4,3}, {71,5,6}, {30,39,29}, {39,30,31}, {71,6,74}, {50,33,51}, {33,50,32}, {74,6,7}, {6,5,59}, {52,33,34}, {33,52,51}, {74,7,8}, {7,6,59}, {54,35,14}, {35,54,53}, {74,8,9}, {8,7,58}, {11,75,10}, {75,11,12}, {9,8,56}, {10,75,74}, {9,10,74}, {10,9,55}, {35,30,42}, {30,35,34}, {11,10,55}, {31,49,48}, {49,31,50}, {75,12,73}, {12,11,54}, {42,30,29}, {69,73,26}, {12,13,73}, {13,12,54}, {37,18,19}, {18,37,17}, {14,73,13}, {73,14,15}, {14,13,54}, {37,19,38}, {73,15,16}, {15,14,35}, {38,19,20}, {73,16,26}, {16,15,45}, {17,37,36}, {18,68,70}, {68,18,17}, {39,28,29}, {28,39,27}, {0,17,36}, {17,0,68}, {27,22,42}, {22,27,21}, {18,70,19}, {23,70,24}, {70,23,22}, {19,70,20}, {35,42,47}, {70,22,21}, {23,43,22}, {43,23,24}, {20,21,38}, {21,20,70}, {27,42,28}, {42,22,43}, {24,70,25}, {25,69,26}, {69,25,70}, {43,24,44}, {24,25,44}, {26,16,45}, {25,26,44}, {21,27,39}, {46,44,45}, {44,46,47}, {30,33,32}, {33,30,34}, {58,7,59}, {29,28,42}, {61,62,66}, {62,61,51}, {50,31,32}, {63,51,52}, {51,63,62}, {56,8,57}, {31,30,32}, {53,65,63}, {65,53,55}, {35,53,52}, {15,35,46}, {34,35,52}, {44,47,43}, {40,38,39}, {38,21,39}, {36,37,41}, {39,31,40}, {40,31,41}, {15,46,45}, {42,43,47}, {26,45,44}, {46,35,47}, {59,5,48}, {48,49,60}, {61,66,67}, {49,50,61}, {55,53,64}, {50,51,61}, {55,9,56}, {11,55,54}, {52,53,63}, {53,54,64}, {54,55,64}, {55,56,65}, {57,8,58}, {49,61,67}, {56,57,66}, {49,67,59}, {57,58,66}, {49,59,60}, {58,59,67}, {59,48,60}, {67,66,58}, {66,65,56}, {65,66,62}, {62,63,65}};

	/** variables */
	cv::Mat img_out;
	vector<cv::Point2f> pts_src, pts_dst, pts_out;
	json fld_src, fld_dst;

	encode_image_base64(img_src, encoded_src);
	encode_image_base64(img_dst, encoded_dst);

	/***********************************************************************/
	// Getting FLD results
	// parse & handle FLD results

	// if there's any errors found, handle it and quit.
	bool is_succeeded = jsn_fld.at("is_succeeded");
	int num_results = jsn_fld.at("num_results");
	if(!is_succeeded || num_results != 2 || jsn_fld.at("results").size() != 2){
		fprintf(stderr, "ERROR: %s\n\n", jsn_fld.at("error_msg"));
		return false;
	}

	/***********************************************************************/
    // SOURCE
	img_src.convertTo(img_src, CV_32F);

	/** parse fld results */
	fld_src	= jsn_fld.at("results").at(0);
	if(fld_src.at("landmarks").size() == 0){ return false; }
	parse_points_from_landmarks(fld_src.at("landmarks").at(0), pts_src);

	/***********************************************************************/
	// DESTINATION
	img_dst.convertTo(img_dst, CV_32F);
	/** parse fld results */
	fld_dst	= jsn_fld.at("results").at(1);
	if(fld_dst.at("landmarks").size() == 0){ return false; }
	parse_points_from_landmarks(fld_dst.at("landmarks").at(0), pts_dst);

	/***********************************************************************/
	// OUTPUT

	img_out = img_src.clone(); //cv::Mat::zeros(img_src.size(), CV_32FC3);
	double ratio = 1.0 / NUM_ITERATIONS * target_result_idx;


	float min_x = img_out.cols-1; float max_x = 0;
	float min_y = img_out.rows-1; float max_y = 0;	

	// get the new points for morphed image based on ratio
	for(int j=0; j<pts_src.size(); j++){
		float x = ratio*pts_dst[j].x + (1 - ratio)*pts_src[j].x;
		float y = (1 - ratio)*pts_src[j].y + ratio*pts_dst[j].y;
		pts_out.push_back(Point2f(x, y));

		min_x = min(min_x, x); min_y = min(min_y, y);
		max_x = max(max_x, x); max_y = max(max_y, y);
	}

	int _width = static_cast<int>((max_x - min_x)*1.2);
	int _height = static_cast<int>((max_y - min_y)*1.2);
	cv::Rect _rect(static_cast<int>(min_x*0.85), static_cast<int>(min_y*0.85), _width, _height);

	// add 8 more points
	add_8_more_points(img_src, pts_src);
	add_8_more_points(img_dst, pts_dst);
	add_8_more_points(img_out, pts_out);

	try{

		// get triangles indices from file and resolve coordinates
		for(int k=0; k<SIZE_TRI_LIST; k++){

			int xc = tri_list[k][0]; int yc = tri_list[k][1]; int zc = tri_list[k][2];

			vector<Point2f> tri_src, tri_dst, tri_out;

			tri_dst.push_back(pts_dst[xc]); tri_dst.push_back(pts_dst[yc]); tri_dst.push_back(pts_dst[zc]);
			tri_src.push_back(pts_src[xc]); tri_src.push_back(pts_src[yc]); tri_src.push_back(pts_src[zc]);
			tri_out.push_back(pts_out[xc]); tri_out.push_back(pts_out[yc]); tri_out.push_back(pts_out[zc]);
			
			// morph each triangle in image to form final morphed image
			tri_morphing(tri_src, tri_dst, tri_out, img_src, img_dst, img_out, ratio);
		}
	}catch (std::exception& e){
		std::cerr << "in tcp_connection::face_morph() > tri_morphing " << e.what() << std::endl;
	}

	// set filename (e.g. 01.jpeg, 02.jpeg, ...) 
	// & save the result to the image file
	//crop_valid_area_from_image(img_out, pts_out, img_out);
	//double _resize_ratio = 100.0d / (double)(img_out.cols);
	//cv::resize(img_out, img_out, cv::Size(), _resize_ratio, _resize_ratio, INTER_LANCZOS4);
	
	img_out = img_out(_rect);

	// print base64-encoded strings as return value
	encode_image_base64(img_out, encoded_out);

	/* FOR DEBUGGING
	// save results to file
	cv::Mat img_src_copy = img_src.clone();
	cv::Mat img_dst_copy = img_dst.clone();
	string filename_debug_src = "./result/result0.jpeg";
	write_delaunay(img_src_copy, sdv_src, COLOR_WHITE, filename_debug_src);
	write_triangle_indices_to_file(sdv_src, pts_src);
	string filename_debug_dst = "./result/result1.jpeg";
	write_delaunay(img_dst_copy, sdv_dst, COLOR_WHITE, filename_debug_dst);
	*/

	return true;
}

void face_morph_by_ratio(
	double ratio,
	vector<cv::Point2f> &pts_src, 
	vector<cv::Point2f> &pts_dst, 
	cv::Mat &img_src, 
	cv::Mat &img_dst,
	string &encoded_out
){

	cv::Mat img_out = img_src.clone();
	vector<cv::Point2f> pts_out;

	const int SIZE_TRI_LIST = 142;
	int tri_list[SIZE_TRI_LIST][3] = {
		{40,37,38}, {37,40,41}, {2,72,1}, {72,2,3}, {0,72,68}, {72,0,1}, {31,2,1}, {2,31,48}, {1,0,36}, {31,1,41}, 
		{72,3,4}, {1,36,41}, {71,4,5}, {4,71,72}, {3,2,48}, {4,48,5}, {48,4,3}, {71,5,6}, {30,39,29}, {39,30,31}, 
		{71,6,74}, {50,33,51}, {33,50,32}, {74,6,7}, {6,5,59}, {52,33,34}, {33,52,51}, {74,7,8}, {7,6,59}, {54,35,14},
		{35,54,53}, {74,8,9}, {8,7,58}, {11,75,10}, {75,11,12}, {9,8,56}, {10,75,74}, {9,10,74}, {10,9,55}, {35,30,42}, 
		{30,35,34}, {11,10,55}, {31,49,48}, {49,31,50}, {75,12,73}, {12,11,54}, {42,30,29}, {69,73,26}, {12,13,73}, 
		{13,12,54}, {37,18,19}, {18,37,17}, {14,73,13}, {73,14,15}, {14,13,54}, {37,19,38}, {73,15,16}, {15,14,35}, 
		{38,19,20}, {73,16,26}, {16,15,45}, {17,37,36}, {18,68,70}, {68,18,17}, {39,28,29}, {28,39,27}, {0,17,36}, 
		{17,0,68}, {27,22,42}, {22,27,21}, {18,70,19}, {23,70,24}, {70,23,22}, {19,70,20}, {35,42,47}, {70,22,21}, 
		{23,43,22}, {43,23,24}, {20,21,38}, {21,20,70}, {27,42,28}, {42,22,43}, {24,70,25}, {25,69,26}, {69,25,70}, 
		{43,24,44}, {24,25,44}, {26,16,45}, {25,26,44}, {21,27,39}, {46,44,45}, {44,46,47}, {30,33,32}, {33,30,34}, 
		{58,7,59}, {29,28,42}, {61,62,66}, {62,61,51}, {50,31,32}, {63,51,52}, {51,63,62}, {56,8,57}, {31,30,32}, 
		{53,65,63}, {65,53,55}, {35,53,52}, {15,35,46}, {34,35,52}, {44,47,43}, {40,38,39}, {38,21,39}, {36,37,41}, 
		{39,31,40}, {40,31,41}, {15,46,45}, {42,43,47}, {26,45,44}, {46,35,47}, {59,5,48}, {48,49,60}, {61,66,67}, 
		{49,50,61}, {55,53,64}, {50,51,61}, {55,9,56}, {11,55,54}, {52,53,63}, {53,54,64}, {54,55,64}, {55,56,65}, 
		{57,8,58}, {49,61,67}, {56,57,66}, {49,67,59}, {57,58,66}, {49,59,60}, {58,59,67}, {59,48,60}, {67,66,58}, 
		{66,65,56}, {65,66,62}, {62,63,65}
	};

	float min_x = img_out.cols-1; float max_x = 0;
	float min_y = img_out.rows-1; float max_y = 0;	

	// get the new points for morphed image based on ratio
	for(int j=0; j<pts_src.size(); j++){
		float x = ratio*pts_dst[j].x + (1 - ratio)*pts_src[j].x;
		float y = (1 - ratio)*pts_src[j].y + ratio*pts_dst[j].y;
		pts_out.push_back(Point2f(x, y));

		min_x = min(min_x, x); min_y = min(min_y, y);
		max_x = max(max_x, x); max_y = max(max_y, y);
	}

	// add 8 more points
	add_8_more_points(img_src, pts_src);
	add_8_more_points(img_dst, pts_dst);
	add_8_more_points(img_out, pts_out);

	try{

		// get triangles indices from file and resolve coordinates
		for(int k=0; k<SIZE_TRI_LIST; k++){

			int xc = tri_list[k][0]; int yc = tri_list[k][1]; int zc = tri_list[k][2];

			vector<Point2f> tri_src, tri_dst, tri_out;

			tri_dst.push_back(pts_dst[xc]); tri_dst.push_back(pts_dst[yc]); tri_dst.push_back(pts_dst[zc]);
			tri_src.push_back(pts_src[xc]); tri_src.push_back(pts_src[yc]); tri_src.push_back(pts_src[zc]);
			tri_out.push_back(pts_out[xc]); tri_out.push_back(pts_out[yc]); tri_out.push_back(pts_out[zc]);
			
			// morph each triangle in image to form final morphed image
			tri_morphing(tri_src, tri_dst, tri_out, img_src, img_dst, img_out, ratio);
		}
	}catch (std::exception& e){
		std::cerr << "in tcp_connection::face_morph() > tri_morphing " << e.what() << std::endl;
	}

	// set filename (e.g. 01.jpeg, 02.jpeg, ...) 
	// & save the result to the image file
	//crop_valid_area_from_image(img_out, pts_out, img_out);
	//double _resize_ratio = 100.0d / (double)(img_out.cols);
	//cv::resize(img_out, img_out, cv::Size(), _resize_ratio, _resize_ratio, INTER_LANCZOS4);
	
	// print base64-encoded strings as return value
	encode_image_base64(img_out, encoded_out);
}


/**
 * perform face morphing
 *
 * @param img_src
 * @param img_dst
 * @param jsn_fld
 * @param encoded_src
 * @param encoded_dst
 * @param encoded_out1
 * @param encoded_out2
 */
bool face_morph_prgv(cv::Mat &img_src, cv::Mat &img_dst, json &jsn_fld, string &encoded_src, string &encoded_dst, string &encoded_out1, string &encoded_out2){

	/** variables */
	vector<cv::Point2f> pts_src, pts_dst;
	json fld_src, fld_dst;

	encode_image_base64(img_src, encoded_src);
	encode_image_base64(img_dst, encoded_dst);

	/***********************************************************************/
	// Getting FLD results
	// parse & handle FLD results

	// if there's any errors found, handle it and quit.
	bool is_succeeded = jsn_fld.at("is_succeeded");
	int num_results = jsn_fld.at("num_results");
	if(!is_succeeded || num_results != 2 || jsn_fld.at("results").size() != 2){
		fprintf(stderr, "ERROR: %s\n\n", jsn_fld.at("error_msg"));
		return false;
	}

	/***********************************************************************/
    // SOURCE
	img_src.convertTo(img_src, CV_32F);

	/** parse fld results */
	fld_src	= jsn_fld.at("results").at(0);
	if(fld_src.at("landmarks").size() == 0){ return false; }
	parse_points_from_landmarks(fld_src.at("landmarks").at(0), pts_src);

	/***********************************************************************/
	// DESTINATION
	img_dst.convertTo(img_dst, CV_32F);
	/** parse fld results */
	fld_dst	= jsn_fld.at("results").at(1);
	if(fld_dst.at("landmarks").size() == 0){ return false; }
	parse_points_from_landmarks(fld_dst.at("landmarks").at(0), pts_dst);

	/***********************************************************************/
	// OUTPUT
	face_morph_by_ratio(0.33d, pts_src, pts_dst, img_src, img_dst, encoded_out1);
	face_morph_by_ratio(0.66d, pts_src, pts_dst, img_src, img_dst, encoded_out2);

	/* FOR DEBUGGING
	Scalar COLOR_WHITE(255, 255, 255);
	// save results to file
	cv::Mat img_src_copy = img_src.clone();
	cv::Mat img_dst_copy = img_dst.clone();
	string filename_debug_src = "./result/result0.jpeg";
	write_delaunay(img_src_copy, sdv_src, COLOR_WHITE, filename_debug_src);
	write_triangle_indices_to_file(sdv_src, pts_src);
	string filename_debug_dst = "./result/result1.jpeg";
	write_delaunay(img_dst_copy, sdv_dst, COLOR_WHITE, filename_debug_dst);
	*/

	return true;
}
