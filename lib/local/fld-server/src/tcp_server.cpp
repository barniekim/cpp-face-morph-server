#include <boost/bind.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

#include <tcp_server.h>

tcp_server::tcp_server(size_t isp_size, int port_num, std::string &home_directory, int max_size){
	using boost::asio::ip::tcp;

	// load & initialize models on the memory for FLD
	std::string clnf_model_path = home_directory + "/model/main_clnf_general.txt";
	LandmarkDetector::CLNF *clnf_model = new LandmarkDetector::CLNF(clnf_model_path);

	clnf_model_			= clnf_model;
	max_size_			= max_size;
	isp_				= new io_service_pool(isp_size);
	acceptor_			= new tcp::acceptor(isp_->get_io_service(), tcp::endpoint(tcp::v4(), port_num));

	new_connection_.reset(new tcp_connection(isp_->get_io_service(), max_size_, clnf_model_));
	acceptor_->async_accept(
		new_connection_->socket(),
		boost::bind(&tcp_server::handle_accept, this, boost::asio::placeholders::error)
	);
}

tcp_server::~tcp_server(){
	if(acceptor_ != NULL){
		delete acceptor_;
	}
	if (isp_ != NULL){
		delete isp_;
	}
}

void tcp_server::run(){
	isp_->run();
}

void tcp_server::stop(){
	isp_->stop();
}

void tcp_server::handle_accept(const boost::system::error_code& error){
	using boost::asio::ip::tcp;

	if(error){ return; }

	fprintf(stderr, "session opened\n");

	new_connection_->start();
	new_connection_.reset(new tcp_connection(isp_->get_io_service(), max_size_, clnf_model_));
	acceptor_->async_accept(
		new_connection_->socket(),
		boost::bind(&tcp_server::handle_accept, this,
		boost::asio::placeholders::error)
	);
}