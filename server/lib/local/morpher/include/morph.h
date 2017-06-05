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

#define REQUEST_TYPE_COMPOSABLE     0 
#define REQUEST_TYPE_PROGRESSIVE    10 

/** definitions */

vector<cv::Point2f> get_points_from_file(string filename);
void transform_triangle(vector<cv::Point2f> &tri1, vector<cv::Point2f> &tri2, cv::Mat &src, cv::Mat &output);
cv::Mat getRectForTriangles(cv::Mat img, vector<Point2f> &Tri);
void point_mult(cv::Mat &img, cv::Mat &mask);
void point_mult2(cv::Mat &img, cv::Mat &mask, int rr, int cc);
void tri_morphing(vector<Point2f> &tri_src, vector<Point2f> &tri_dst, vector<Point2f> &tri_out, cv::Mat &img1, cv::Mat &img2, cv::Mat &img, double imageRatio);

bool contain_tri_in_rect(Rect r, Vec6f t);
void parse_points_from_landmarks(json landmarks, vector<cv::Point2f> &points);
void points_to_subdiv(vector<cv::Point2f> points, cv::Subdiv2D &sdv);
void get_json_from_file(const char *filename, json &j);
void encode_image_base64(cv::Mat mat, std::string &encoded);
void crop_valid_area_from_image(cv::Mat &img_in, cv::Mat &img_out);

int get_index_from_points_vector(vector<cv::Point2f> points, cv::Point2f p);
void write_triangle_indices_to_file(cv::Subdiv2D &sdv, vector<cv::Point2f> points);
static void write_delaunay(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color, std::string filename);

void generate_random_prefix(char *s, const int len);
void add_8_more_points(cv::Mat &img, vector<cv::Point2f> &pts);

/** 
 * main function of face morphing feature: 
 * morphing (composable) 
 */
bool face_morph(cv::Mat &img_src, cv::Mat &img_dst, json &jsn_fld, int target_result_idx, string &encoded_src, string &encoded_dst, string &encoded_out);

/** morphing (progressive) */
bool face_morph_prgv(cv::Mat &img_src, cv::Mat &img_dst, json &jsn_fld, string &encoded_src, string &encoded_dst, string &encoded_out1, string &encoded_out2);
