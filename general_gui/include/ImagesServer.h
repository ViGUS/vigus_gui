#ifndef IMAGESSERVER_H
#define IMAGESSERVER_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace ims {
    class ServerImages {
    enum class eType { UDP, TCP };

    template<eType Type_>

    public:
        ServerImages(int _Port);
        void send(cv::Mat _img, int _percent);

    private:
        boost::asio::ip::tcp::socket *mSocketTCP;
        boost::asio::ip::udp::socket *mSocketUDP;
        // 666 TODO: gather following variables with traits
		std::vector<boost::asio::ip::tcp::socket*> mTcpConnection;

        bool mRun = false;
        
    };
}

#include <ImagesServer.inl>

#endif // IMAGESSERVER_H