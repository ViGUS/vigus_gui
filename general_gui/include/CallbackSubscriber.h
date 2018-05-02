//
//
//
//
//
//

#ifndef CBS_CALLBACKSUBSCRIBER_H_
#define CBS_CALLBACKSUBSCRIBER_H_

#include <string>
#include <mutex>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cbs {

	template<typename CallbackType_>
	class CallbackSubscriber {
	public:
		CallbackSubscriber(std::string _name);

		void receiveDataCallback(const CallbackType_ &_buffer);

		template<typename ReceiveDataType_>
		bool receiveData(ReceiveDataType_ &_data);

		void stop(){ mRun = false; }
	private:
		pcl::PointCloud<pcl::PointXYZRGB> *mPointCloud;
		bool mRecData = false;
		bool mRun = false;
		std::mutex mSafeGuard;

		std::string mName;

	};


}

#include <CallbackSubscriber.inl>

#endif