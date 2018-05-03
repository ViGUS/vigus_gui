#ifndef IMAGESCLIENT_H
#define IMAGESCLIENT_H

#include <string>
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

namespace imc {
    class ImagesClient {
    enum class eType { UDP, TCP };

    template<eType Type_>
    
    public:
        ImagesClient(int _port, std::string _ip);
        void retrieve(cv::Mat &_img);

    private:
        std::vector<unsigned char> mCompressImage;
        tcp::socket *mSocketTCP;
        udp::socket *mSocketUDP;
        
        // 666 TODO: gather following variables with traits
		std::vector<boost::asio::ip::tcp::socket*> mTcpConnection;
			
		std::vector<boost::asio::ip::udp::endpoint*> mUdpConnection;
    };
}

#include <ImagesClient.inl>

#endif // IMAGESCLIENT_H