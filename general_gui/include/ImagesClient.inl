#include <ImagesClient.h>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

namespace imc {

    template<>
    ClientImages<Type::UDP>::ClientImages(std::string _ip, int _port)
    {
        std::cout << "Trying connection to server "<< _ip << " in port " <<_port  << std::endl;
        boost::asio::io_service io_service;
		boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::address::from_string(_ip), _port);

		mSocketUDP = new udp::socket (io_service);
		mSocketUDP->open(udp::v4());

        std::cout << "Client socket created" << std::endl;
        mUdpConnection.push_back(endpoint);

    }

    template<>
    void ClientImages<Type::UDP>::retrieve(cv::Mat &_img)
    {   
        unsigned char buffer[1024];
        int sizePicture;
        mCompressImage.clear();
        std::string sinc = "takePicture";
        // std::cout << "Sending petition to take a picture" << std::endl;
		boost::system::error_code ignored_error;
        boost::array<char, sizeof(sinc)> send_buffer;
		memcpy(&send_buffer[0], &sinc, sizeof(sinc))
		try {
			mSocketUDP->send_to(boost::asio::buffer(send_buffer), *mUdpConnection, 0, ignored_error);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

        boost::array<char, sizeof(int)> recv_buf;
		udp::endpoint sender_endpoint;
        try {
			size_t length = mSocketUDP->receive_from(boost::asio::buffer(recv_buf), sender_endpoint);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}
        int sizePicture;
		memcpy(&sizePicture, &recv_buf[0], sizeof(sizePicture));
        //std::cout << sizePicture << std::endl;

        // std::cout << "Size buffer received, sending confirmation" << std::endl;
        boost::array<char, sizeof(sinc2)> send_buffer2;
		memcpy(&send_buffer2[0], &sinc2, sizeof(sinc2))
		try {
			mSocketUDP->send_to(boost::asio::buffer(send_buffer2), *mUdpConnection, 0, ignored_error);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

        size_t len;
        do{

            boost::array<char, 1024> buffer;
		    udp::endpoint send_endpoint;
            try {
		    	len = mSocketUDP->receive_from(boost::asio::buffer(buffer), sender_endpoint);
		    }
		    catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            for (int i = 0; i < static_cast<int>(len); i++){
                mCompressImage.push_back(buffer[i]);
            }
            if(static_cast<int>(len) == -1){
                break;
            }
        }while(mCompressImage.size() != sizePicture);

        /* decodificar imagen*/
        // std::cout << mCompressImage.size() << std::endl;
        try {
            _img = cv::imdecode(mCompressImage, 1);
        }
        catch (cv::Exception e){
            std::cout << "error descomprimiendo imagen" << std::endl;
        }
    }

    template<>
    ClientImages<Type::TCP>::ClientImages(std::string _ip, int _port)
    {
        std::cout << "Trying connection to server "<< _ip << " in port " <<_port  << std::endl;
        boost::asio::io_service io_service;
		boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(_ip), _port);

		mSocketTCP = new tcp::socket (io_service);
        mSocketTCP->connect(endpoint);

        std::cout << "Client socket created" << std::endl;
        mTcpConnection.push_back(endpoint);

    }

    template<>
    void ClientImages<Type::TCP>::retrieve(cv::Mat &_img)
    {   
        unsigned char buffer[1024];
        int sizePicture;
        mCompressImage.clear();
        std::string sinc = "takePicture";
        // std::cout << "Sending petition to take a picture" << std::endl;
		boost::system::error_code error;
		try {
            if (mTcpConnection->is_open())
				boost::asio::write(*mTcpConnection, boost::asio::buffer(sinc), error);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

        boost::array<char, sizeof(int)> recv_buf;
		udp::endpoint sender_endpoint;
        try {
			size_t length = mSocketTCP->read_some(boost::asio::buffer(recv_buf), error);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}
        int sizePicture;
		memcpy(&sizePicture, &recv_buf[0], sizeof(sizePicture));
        //std::cout << sizePicture << std::endl;

        // std::cout << "Size buffer received, sending confirmation" << std::endl;
		try {
            if (mTcpConnection->is_open())
				boost::asio::write(*mTcpConnection, boost::asio::buffer(sinc2), ignored_error);
		}
		catch (std::exception &e) {
			std::cerr << e.what() << std::endl;
		}

        size_t len;
        do{

            boost::array<char, 1024> buffer;
		    udp::endpoint send_endpoint;
            try {
		    	len = mSocketTCP->read_some(boost::asio::buffer(buffer), error);
		    }
		    catch (std::exception &e) {
		    	std::cerr << e.what() << std::endl;
		    }

            for (int i = 0; i < static_cast<int>(len); i++){
                mCompressImage.push_back(buffer[i]);
            }
            if(static_cast<int>(len) == -1){
                break;
            }
        }while(mCompressImage.size() != sizePicture);

        /* decodificar imagen*/
        // std::cout << mCompressImage.size() << std::endl;
        try {
            _img = cv::imdecode(mCompressImage, 1);
        }
        catch (cv::Exception e){
            std::cout << "error descomprimiendo imagen" << std::endl;
        }
    }

}