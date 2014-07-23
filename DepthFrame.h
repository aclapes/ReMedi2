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
	void getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud);
    
    void setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals);
    void getNormals(pcl::PointCloud<pcl::Normal>& normals);
    
    typedef boost::shared_ptr<DepthFrame> Ptr;
    
private:
	cv::Mat m_UIDMat;		// Here the player indices

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloud;
    pcl::PointCloud<pcl::Normal>::Ptr m_pNormals;
};

