#include "DepthFrame.h"

#include <pcl/common/transforms.h>

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
        m_ReferencePoint = rhs.m_ReferencePoint;
        m_T = rhs.m_T;
    }
    
	return *this;
}

void DepthFrame::getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    cloud = *m_pCloud;
}

void DepthFrame::getPointCloud(cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    MatToPointCloud(Frame::get(), mask, cloud);
}

void DepthFrame::setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals)
{
    m_pNormals = pNormals;
}

void DepthFrame::getNormals(pcl::PointCloud<pcl::Normal>& normals)
{
    normals = *m_pNormals;
}

void DepthFrame::setReferencePoint(pcl::PointXYZ reference)
{
    m_ReferencePoint = reference;
}

pcl::PointXYZ DepthFrame::getReferencePoint()
{
    return m_ReferencePoint;
}

void DepthFrame::setRegistrationTransformation(Eigen::Matrix4f T)
{
    m_T = T;
}

Eigen::Matrix4f DepthFrame::getRegistrationTransformation()
{
    return m_T;
}

pcl::PointXYZ DepthFrame::getRegisteredReferencePoint()
{
    // Register the reference point
    pcl::PointXYZ regReferencePoint;
    
    regReferencePoint.getVector4fMap() = (m_T * m_ReferencePoint.getVector4fMap());
    
    return regReferencePoint;
}

void DepthFrame::getRegisteredPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::transformPointCloud(*m_pCloud, cloud, m_T);
}

void DepthFrame::getRegisteredAndReferencedPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    pcl::PointXYZ regReferencePoint = getRegisteredReferencePoint();
    
    // Build a translation matrix to the registered reference the cloud after its own registration
    Eigen::Matrix4f E = Eigen::Matrix4f::Identity();
    E.col(3) = regReferencePoint.getVector4fMap(); // set translation column
    
    // Apply registration first and then referenciation (right to left order in matrix product)
    pcl::transformPointCloud(*m_pCloud, cloud, E.inverse() * m_T);
}

void DepthFrame::getDeregisteredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pRegisteredCloud, pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    Eigen::Matrix4f iT = m_T.inverse();
    pcl::transformPointCloud(*pRegisteredCloud, cloud, iT);
}