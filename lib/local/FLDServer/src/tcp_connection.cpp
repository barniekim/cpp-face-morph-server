#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/bind.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <assert.h>
#include <dirent.h>
#include <ctime>

#include <io_service_pool.h>
#include <tcp_connection.h>

// morphing library
#include <morph.h>

// json library
#include <json.hpp>

using json = nlohmann::json;
using boost::asio::ip::tcp;

tcp::socket& tcp_connection::socket(){
	return socket_;
}

void convert_to_grayscale(const cv::Mat& in, cv::Mat& out){
	if(in.channels() == 3){
		// Make sure it's in a correct format
		if(in.depth() != CV_8U){
			if(in.depth() == CV_16U){
				cv::Mat tmp = in / 256;
				tmp.convertTo(tmp, CV_8U);
				cv::cvtColor(tmp, out, CV_BGR2GRAY);
			}
		}else{
			cv::cvtColor(in, out, CV_BGR2GRAY);
		}
	}else if(in.channels() == 4){
		cv::cvtColor(in, out, CV_BGRA2GRAY);
	}else{
		if(in.depth() == CV_16U){
			cv::Mat tmp = in / 256;
			out = tmp.clone();
		}else if(in.depth() != CV_8U){
			in.convertTo(out, CV_8U);
		}else{
			out = in.clone();
		}
	}
}

void tcp_connection::start(){
	socket_.async_read_some
		(boost::asio::buffer(&data_len_, sizeof(data_len_)),
		boost::bind(&tcp_connection::handle_read_datasize, 
			shared_from_this(),
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}

void tcp_connection::handle_read_datasize(const boost::system::error_code& error, size_t bytes_transferred){

	if (error == boost::asio::error::eof) {
		fprintf(stderr, "boost::asio::error::eof\n");
		boost::system::error_code ignored_ec;
		socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
		return;
	}else{
		if(error){
			fprintf(stderr, "error in reading the data size\n");
			throw boost::system::system_error(error);
		}
	}

	assert(bytes_transferred == sizeof(data_len_));
	fprintf(stderr, "data_len = %d\n", data_len_);

	if (data_len_ > 0 && data_len_ < 1024 * 1024 * 32) {
		if (data_) delete[] data_;
		data_ = new unsigned char [data_len_];
		data_offset_ = 0;

		start_read();
	}else {
		// sanity-check
		using namespace std;

		string str_json = string("{'is_succeeded':false,'error_msg':''}");
		write_result(str_json);
	}
}

void tcp_connection::stop(){
	if (data_) delete[] data_;
	data_ = NULL;
	data_len_ = 0;
}

tcp_connection::tcp_connection(boost::asio::io_service& io_service, int max_size, LandmarkDetector::CLNF *clnf_model) : socket_(io_service){
	max_size_			= max_size;
	data_				= NULL;
	data_offset_		= 0;
	clnf_model_			= clnf_model;
}

void tcp_connection::start_read(){
	socket_.async_read_some(
		boost::asio::buffer(data_ + data_offset_, data_len_ - data_offset_),
		boost::bind(&tcp_connection::handle_read, shared_from_this(),
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred)
	);
}

void tcp_connection::start_write(){
	boost::asio::async_write(
		socket_, boost::asio::buffer(data_ + data_offset_, data_len_ - data_offset_),
		boost::bind(&tcp_connection::handle_write, shared_from_this(),
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred)
	);
}

void tcp_connection::sync_write(char* data_to_send, int len_to_send){
	int offset = 0;
	do{
		int nSent = boost::asio::write(socket_, boost::asio::buffer(data_to_send + offset, len_to_send - offset));
		offset += nSent;
	}while (offset < len_to_send);
}

void tcp_connection::write_result(std::string& result){
	int _size = result.length() + 1;
	sync_write((char *)&_size, sizeof(_size));
	sync_write((char *)result.c_str(), _size);
	fprintf(stderr, "sent: %d bytes\n\n", (int)(sizeof(_size) + _size));
}

/**
 * tcp_connection::Process()
 * 
 * @params
 * - 4 bytes; request_type (int); 0-COMPOSABLE, 1-PROGRESSIVE
 * - 4 bytes; target_result_idx (int)
 * - 4 bytes; for total size
 * - 4 x #-files bytes; for filesize (input/target)
 * - total filesize; for file data
 *
 */
void tcp_connection::Process(){

	int s_time = clock();
	int e_time = s_time;

	assert(data_len_ >= 1);

	vector<cv::Rect_<double> > bounding_boxes;
	LandmarkDetector::FaceModelParameters det_parameters;

	// get & extract data from the socket
	vector<string> files;
	unsigned char* data_ptr = data_;
	
	// request_type
	uint32_t request_type;
	memcpy(&request_type, data_ptr, sizeof(uint32_t));
	data_ptr += sizeof(uint32_t);

	// target_result_idx
	uint32_t target_result_idx;
	memcpy(&target_result_idx, data_ptr, sizeof(uint32_t));
	data_ptr += sizeof(uint32_t);

	// num_files
	uint32_t num_files;
	memcpy(&num_files, data_ptr, sizeof(uint32_t));
	fprintf(stderr, "- num_files: %d\n", num_files);
	data_ptr += sizeof(uint32_t);

	/// create temp image files
	char cr_tmp_prefix[18];
	generate_random_prefix(cr_tmp_prefix, 15);
	std::string tmp_prefix(cr_tmp_prefix);
	const boost::filesystem::path path("/root/tmp/" + tmp_prefix);
	boost::filesystem::create_directories(path);

	for(int i=0; i<num_files; i++){
		fprintf(stderr, "\t- file-idx: %d\n", i);
		uint32_t _file_len;
		memcpy(&_file_len, data_ptr, sizeof(uint32_t));
		fprintf(stderr, "\t\t- file length %d\n", _file_len);
		if(_file_len > 0){
			data_ptr += sizeof(uint32_t);

			char _file_data[_file_len];
			memcpy(_file_data, data_ptr, _file_len);

			ofstream _of_stream;
			std::string _filename = "/root/tmp/"+tmp_prefix+"/_file"+std::to_string(i)+".jpg";
			_of_stream.open(_filename, ios::binary|ios::out);
			_of_stream.write((const char*) _file_data, _file_len);
			_of_stream.close();
			data_ptr += _file_len;

			FILE *p_file = fopen(_filename.c_str(), "rb");
			fseek(p_file, 0, SEEK_END);
			int _size = ftell(p_file);
			fclose(p_file);
			fprintf(stderr, "\t\t- file size %d\n", _size);
			if(_size == _file_len){
				files.push_back(_filename);
			}
		}
	}
	fprintf(stderr, "- # of files loaded: %d\n\n", files.size());

	// process an image at a time with the loop below
	json j_output;
	vector<json> j_files;
	cv::Mat img_src;
	cv::Mat img_dst;
	for(size_t file_idx = 0; file_idx < files.size(); file_idx++){
		string file = files.at(file_idx);

		fprintf(stderr, "\t\t file_idx: %d\n", file_idx);
		fprintf(stderr, "\t\t file_name: %s\n", file.c_str());

		// load image: read_image -> grayscaled_image
		cv::Mat read_image = cv::imread(file, -1);
		if (read_image.empty()){
			cout << "Could not read the input image" << endl;
			return;
		}
		if(read_image.cols > 200){
			double _resize_ratio = 200.0d / (double)(read_image.cols);
			cv::resize(read_image, read_image, cv::Size(), _resize_ratio, _resize_ratio, INTER_LANCZOS4);
		}
		cv::Mat_<float> depth_image;
		cv::Mat_<uchar> grayscale_image;
		convert_to_grayscale(read_image, grayscale_image);
		if(file_idx == 0){
			img_src = read_image;
		}else{
			img_dst = read_image;
		}
		fprintf(stderr, "\t\t\t image converted.\n");

		/** 1) Face Detection (FD) */
		vector<cv::Rect_<double>> face_rects;
		vector<double> confidences;

		dlib::frontal_face_detector face_detector_hog = dlib::get_frontal_face_detector();
		LandmarkDetector::DetectFacesHOG(face_rects, grayscale_image, face_detector_hog, confidences);
		fprintf(stderr, "\t\t\t face detected:%d\n", face_rects.size());

		/** 2) Face Landmark Detection (FLD) */
		vector<json> _faces;
		vector<vector<json>> _landmarks;
		if(face_rects.size()> 0){

			int num_faces = 0;
			for(size_t face_idx=0; face_idx < face_rects.size(); ++face_idx){
				cv::Rect_<double> _face_rect = face_rects[face_idx];
				vector<json> _landmark;

				// store face rect
				json j_face = { json::array({"x", _face_rect.x}), json::array({"y", _face_rect.y}), json::array({"width", _face_rect.width}), json::array({"height", _face_rect.height}) };
				_faces.push_back(j_face);
				num_faces++;

				if(LandmarkDetector::DetectLandmarksInImage(grayscale_image, _face_rect, *clnf_model_, det_parameters)){
					// store landmarks
					int n = clnf_model_->patch_experts.visibilities[0][0].rows;
					for (int i = 0; i < n; ++i){
						json j_point = { json::array({"x", (clnf_model_->detected_landmarks.at<double>(i) + 1)}), json::array({"y", (clnf_model_->detected_landmarks.at<double>(i+n) + 1)}) };
						_landmark.push_back(j_point);
					}
					_landmarks.push_back(_landmark);
				}else{
					//// CASE: FLD FAILED
					break;
				}
				break;
			}
			fprintf(stderr, "\t\t\t face landmarks detection: %d\n\n", _landmarks.size());
		}

		// set JSON-result
		json j_file;
		j_file["filename"]	= file;
		j_file["width"]		= read_image.cols;
		j_file["height"]	= read_image.rows;
		j_file["landmarks"] = _landmarks;
		j_file["faces"] = _faces;
		j_files.push_back(j_file);
	}

	/** build up the json strings to be printed */
	j_output["num_results"]		= files.size();
	j_output["results"]			= j_files;
	j_output["is_succeeded"]	= true;
	j_output["error_msg"]		= "";

	fprintf(stderr, "FLD done. \n\n");

	/** 3) Face Morphing */	
	fprintf(stderr, "Face Morphing type: %d\n\n", request_type);
	if(request_type == 0){
		
		// COMPOSABLE
		string encoded_src, encoded_dst, encoded_out;
		bool is_morphed = face_morph(img_src, img_dst, j_output, target_result_idx, encoded_src, encoded_dst, encoded_out);

		j_output["is_morphed"]		= is_morphed;
		j_output["encoded_src"]		= encoded_src;
		j_output["encoded_dst"]		= encoded_dst;
		j_output["encoded_out"]		= encoded_out;

	}else if(request_type == 1){

		// PROGRESSIVE
		string encoded_src, encoded_dst, encoded_out1, encoded_out2;
		bool is_morphed = face_morph_prgv(img_src, img_dst, j_output, encoded_src, encoded_dst, encoded_out1, encoded_out2);

		j_output["is_morphed"]		= is_morphed;
		j_output["encoded_src"]		= encoded_src;
		j_output["encoded_dst"]		= encoded_dst;
		j_output["encoded_out1"]	= encoded_out1;
		j_output["encoded_out2"]	= encoded_out2;
	}

	std::string result = j_output.dump();
	fprintf(stderr, "result: %s\n", result.c_str());
	write_result(result);

	e_time = clock();
	fprintf(stderr, "\nExecution time: %.3f\n\n", (e_time-s_time)/double(CLOCKS_PER_SEC));

	/// remove temp image files
	boost::filesystem::remove_all(path);

	try {
		stop();
		boost::system::error_code ignored_ec;
		socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ignored_ec);
	}catch (std::exception& e){
		std::cerr << "in tcp_connection::Process() " << e.what() << std::endl;

		// print out the error message
		json j_output;
		j_output["is_succeeded"]	= false;
		j_output["error_msg"]			= "ERROR";
		std::string result = j_output.dump();
		write_result(result);
	}
}

void tcp_connection::handle_read(const boost::system::error_code& error, size_t bytes_transferred){
	if(error){ return; }
	data_offset_ += bytes_transferred;
	//fprintf(stderr, "handle_read. data_offset: %d\n", data_offset_);
	if (data_offset_ < data_len_) {
		//fprintf(stderr, "not enough data passed in... start_read() again.\n");
		start_read();
	}else {
		Process();
	}
}

void tcp_connection::handle_write(const boost::system::error_code& error, size_t bytes_transferred){
	if(error){ return; }
	data_offset_ += bytes_transferred;
	if (data_offset_ < data_len_) {
		start_write();
	}else {
		data_offset_ = 0;
	}
}
