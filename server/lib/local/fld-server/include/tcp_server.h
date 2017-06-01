#pragma once

#include <boost/asio.hpp>

#include "LandmarkCoreIncludes.h"

// System includes
#include <fstream>
#include <ctime>

// Boost includes
#include <filesystem.hpp>
#include <filesystem/fstream.hpp>

// tbb
#include <tbb/tbb.h>

#include <io_service_pool.h>
#include <tcp_connection.h>

class tcp_server{
public:
	tcp_server(size_t isp_size, int port_num, std::string &home_directory, int max_size);
	~tcp_server();
	void run();
	void stop();

private:
	void handle_accept(const boost::system::error_code& error);

	boost::asio::ip::tcp::acceptor* acceptor_;
	io_service_pool* isp_;
	tcp_connection::pointer new_connection_;
	int max_size_;

	LandmarkDetector::CLNF *clnf_model_;
};
