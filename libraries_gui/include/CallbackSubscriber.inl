//
//
//
//
//
//

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
	//-----------------------------------------------------------------------------------------------------------------
	// PRIVATE CLASS
	//-----------------------------------------------------------------------------------------------------------------
	// PointCloud
	//-----------------------------------------------------------------------------------------------------------------
	template<>
	void CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr>::receiveDataCallback(const sensor_msgs::PointCloud2ConstPtr &_buffer) {
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mReceivedPCL(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		auto size = (_buffer->width * _buffer->height);
		if(size == 0){
			return;
		}

		pcl::fromROSMsg(*_buffer, *mReceivedPCL);
		mSafeGuard.lock();
		mPointCloud = new pcl::PointCloud<pcl::PointXYZRGB>(*mReceivedPCL);
		mSafeGuard.unlock();
		mRecData = true;
		
	}

	//-----------------------------------------------------------------------------------------------------------------
	template<>
	CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr>::CallbackSubscriber(std::string _name) {
		mName = _name;
		mRun = true;
		std::cout << "Creating Ros subscriber..." << std::endl;
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe(mName, 1, &CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr>::receiveDataCallback, this);
	}	

	template<>
	template<>
	bool CallbackSubscriber<sensor_msgs::PointCloud2ConstPtr>::receiveData(pcl::PointCloud<pcl::PointXYZRGB> &_data) {
		
		if(mRecData == true){
			mSafeGuard.lock();
			_data = *mPointCloud;
			mSafeGuard.unlock();
			return true;
		}else{
			return false;
		}
		
		
	}
	
}