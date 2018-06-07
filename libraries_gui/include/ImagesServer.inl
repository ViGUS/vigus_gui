#include <ImagesServer.h>


namespace ims {

    template<>
    ServerImages<Type::UDP>::ServerImages(int _port) {

        std::cout << "Initialized server in ip 0.0.0.0 and port " << _port << std::endl;
        boost::asio::io_service io_service;
		mSocketUDP = new boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), _port));
        std::cout << "Server socket created"<< std::endl;
        mRun = true;
    }

    template<>
    void ServerImages<Type::UDP>::send(cv::Mat _img, int _percent) {

        boost::array<char, 1024> buffer;

        if(mRun){
            // std::cout << "Waiting for take a picture"<< std::endl;
            boost::asio::ip::udp::endpoint *remote_endpoint = new boost::asio::ip::udp::endpoint();
			boost::system::error_code error;
            try {
                mSocketUDP->receive_from(boost::asio::buffer(buffer), *remote_endpoint, 0, error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }
            // std::cout << "Petition received. Sending picture"<< std::endl;

            std::vector<uchar> buff;
            std::vector<int> params;
            params.push_back(CV_IMWRITE_JPEG_QUALITY);
            params.push_back(_percent);
            cv::imencode(".jpg", _img, buff, params);
            int bufferSize = buff.size();
            // std::cout << bufferSize << std::endl;

            boost::system::error_code ignored_error;
            boost::array<char, sizeof(int)> send_buffer;
			memcpy(&send_buffer[0], &bufferSize, sizeof(int));
            try {
                mSocketUDP->send_to(boost::asio::buffer(send_buffer), *remote_endpoint, 0, ignored_error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            try {
                mSocketUDP->receive_from(boost::asio::buffer(buffer), *remote_endpoint, 0, error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            boost::array<char, sizeof(buff)> send_buffer2;
			memcpy(&send_buffer2[0], &buff, sizeof(buff));
            try {
                mSocketUDP->send_to(boost::asio::buffer(send_buffer2), *remote_endpoint, 0, ignored_error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

        }

    }

    template<>
    ServerImages<Type::TCP>::ServerImages(int _port) {

        std::cout << "Initialized server in ip 0.0.0.0 and port " << _port << std::endl;
        boost::asio::io_service io_service;
		boost::asio::ip::tcp::acceptor acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), _port));
        mSocketTCP = new boost::asio::ip::tcp::socket(io_service);
		acceptor.accept(*mSocketTCP);
        mTcpConnections.push_back(mSocketTCP);
        std::cout << "Server socket created" << std::endl;
        mRun = true;
    }

    template<>
    void ServerImages<Type::TCP>::send(cv::Mat _img, int _percent) {

        unsigned char buffer[1024];

        boost::array<char, 1024> recv_buf;

        if(mRun){
            // std::cout << "Waiting for take a picture"<< std::endl;
			boost::system::error_code error;
            size_t length;
            try {
                length = mSocketTCP->read_some(boost::asio::buffer(recv_buf), error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }
            // std::cout << "Petition received. Sending picture"<< std::endl;

            std::vector<uchar> buff;
            std::vector<int> params;
            params.push_back(CV_IMWRITE_JPEG_QUALITY);
            params.push_back(_percent);
            cv::imencode(".jpg", _img, buff, params);
            int bufferSize = buff.size();
            // std::cout << bufferSize << std::endl;

            std::string send_buffer;
            memcpy(&send_buffer[0], &bufferSize, sizeof(int));
            try {
            if (mTcpConnection->is_open())
				boost::asio::write(*mTcpConnection, boost::asio::buffer(send_buffer), error);
		    }
		    catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            try {
                length = mSocketTCP->read_some(boost::asio::buffer(recv_buf), error);
            }
            catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            std::string send_buffer2;
            memcpy(&send_buffer2[0], &buff, sizeof(buff));
            try {
            if (mTcpConnection->is_open())
				boost::asio::write(*mTcpConnection, boost::asio::buffer(send_buffer2), error);
		    }
		    catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

        }

    }

}