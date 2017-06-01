#pragma once

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/asio.hpp>

#include "LandmarkCoreIncludes.h"

// OpenCV includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

// System includes
#include <fstream>
#include <ctime>

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

// tbb
#include <tbb/tbb.h>

typedef long long __int64;
typedef unsigned long long __uint64;

#define ERROR_OK	0


class tcp_connection: public boost::enable_shared_from_this<tcp_connection>, private boost::noncopyable {
public:
	typedef boost::shared_ptr<tcp_connection> pointer;

	static pointer create(boost::asio::io_service& io_service, int max_size, LandmarkDetector::CLNF *clnf_model){
		return pointer(new tcp_connection(io_service, max_size, clnf_model));
	}

	boost::asio::ip::tcp::socket& socket();
	void start();
	void stop();
	tcp_connection(boost::asio::io_service& io_service, int max_size, LandmarkDetector::CLNF *clnf_model);

private:
	void start_read();
	void start_write();
	
	void sync_write(char* data_to_send, int len_to_send);
	void write_result(std::string& xml);
	void process_fldm();
	
	void handle_read_datasize(const boost::system::error_code& error, size_t bytes_transferred);
	void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
	void handle_write(const boost::system::error_code& error, size_t bytes_transferred);

	boost::asio::ip::tcp::socket socket_;
	int data_len_;
	unsigned char* data_;
	int data_offset_;
	int max_size_;
	LandmarkDetector::CLNF *clnf_model_;
};

