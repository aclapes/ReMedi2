#include "DepthFrame.h"

DepthFrame::DepthFrame() : Frame()
{
}

DepthFrame::DepthFrame(cv::Mat mat) : Frame(mat / 8), m_pCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	// Player index map
    cv::Mat bitmask (mat.rows, mat.cols, mat.type(), cv::Scalar(7));
    cv::bitwise_and(mat, bitmask, m_UIDMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
    
    // Create a point cloud
    MatToPointCloud(Frame::get(), *m_pCloud);
}

DepthFrame::DepthFrame(cv::Mat mat, cv::Mat mask) : Frame(mat / 8, mask), m_pCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
	// Player index map
    cv::Mat bitmask (mat.rows, mat.cols, mat.type(), cv::Scalar(7));
    cv::bitwise_and(mat, bitmask, m_UIDMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
    
    // Create a point cloud
    MatToPointCloud(Frame::getMasked(), *m_pCloud);
}

DepthFrame::DepthFrame(const DepthFrame& rhs) : Frame(rhs)
{
    *this = rhs;
}

DepthFrame& DepthFrame::operator=(const DepthFrame& rhs)
{
    if (this != &rhs)
    {
        Frame::operator=(rhs);
        m_UIDMat = rhs.m_UIDMat;
        m_pCloud = rhs.m_pCloud;
        m_pNormals = rhs.m_pNormals;
    }
    
	return *this;
}

void DepthFrame::getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud = *m_pCloud;
}

void DepthFrame::setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals)
{
    m_pNormals = pNormals;
}

void DepthFrame::getNormals(pcl::PointCloud<pcl::Normal>& normals)
{
    normals = *m_pNormals;
}