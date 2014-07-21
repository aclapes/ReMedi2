#pragma once

#include "Frame.h"

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// OpenCV <-> PCL
#include "conversion.h"

#include "ColorFrame.h"

#include <boost/shared_ptr.hpp>

class DepthFrame : public Frame
{
public:
	// Constructors
	DepthFrame();
	DepthFrame(cv::Mat);
	DepthFrame(cv::Mat, cv::Mat);
	DepthFrame(const DepthFrame&);

	// Operators
	DepthFrame& operator=(const DepthFrame& other);

	// Methods
	cv::Mat getUserFreeDepthMap(cv::Mat&);
	void getPointCloud(pcl::PointCloud<pcl::PointXYZ>&);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud();
    void getColoredPointCloud(ColorFrame cframe, pcl::PointCloud<pcl::PointXYZRGB>& cloud);
	void getForegroundPointCloud(pcl::PointCloud<pcl::PointXYZ>&);
    void getForegroundPointCloud(cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>&, bool combined = true);
	void getUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>&);
	void getForegroundUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>&);
    void setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals);
    pcl::PointCloud<pcl::Normal>::Ptr getNormals();
    void getNormals(pcl::PointCloud<pcl::Normal>& normals);
        
    typedef boost::shared_ptr<DepthFrame> Ptr;
    
private:
	cv::Mat m_UIDMat;		// Here the player indices

    // Normals
    pcl::PointCloud<pcl::Normal>::Ptr m_pNormals;
};

