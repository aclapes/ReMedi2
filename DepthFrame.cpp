#include "DepthFrame.h"

DepthFrame::DepthFrame() : Frame()
{
}

DepthFrame::DepthFrame(cv::Mat mat) : Frame(mat / 8)
{
	// Player index map
    cv::Mat bitmask (mat.rows, mat.cols, mat.type(), cv::Scalar(7));
    cv::bitwise_and(mat, bitmask, m_UIDMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
}

DepthFrame::DepthFrame(cv::Mat mat, cv::Mat mask) : Frame(mat / 8, mask)
{
	// Player index map
    cv::Mat bitmask (mat.rows, mat.cols, mat.type(), cv::Scalar(7));
    cv::bitwise_and(mat, bitmask, m_UIDMat);
	m_UIDMat.convertTo(m_UIDMat, CV_8UC1);
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
        m_pNormals = rhs.m_pNormals;
    }
    
	return *this;
}

cv::Mat DepthFrame::getUserFreeDepthMap(cv::Mat& ufDepthMap)
{
	cv::Mat mask;
	cv::threshold(m_UIDMat, mask, 0, 255, CV_THRESH_BINARY_INV); // users' pixels to 0, non-users' ones to 255.

	m_Mat.copyTo(ufDepthMap, mask);

	return ufDepthMap;
}


void DepthFrame::getPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	MatToPointCloud(m_Mat, cloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DepthFrame::getPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZ>());
    MatToPointCloud(m_Mat, *pCloud);
	return pCloud;
}

void DepthFrame::getColoredPointCloud(ColorFrame cframe, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    cv::Mat colorMat = cframe.get();
	MatToColoredPointCloud(m_Mat, colorMat, cloud);
}


void DepthFrame::getForegroundPointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	cv::Mat fgProjDepthMat;
	m_Mat.copyTo(fgProjDepthMat, m_Mask);

	MatToPointCloud(fgProjDepthMat, cloud);
}


void DepthFrame::getForegroundPointCloud(cv::Mat mask, pcl::PointCloud<pcl::PointXYZ>& cloud, bool combined)
{
	cv::Mat fgProjDepthMat;
    
    if (combined)
    {
        cv::Mat tmp, combinedMask;
        mask.convertTo(tmp, CV_8UC1);
        cv::bitwise_and(m_Mask, tmp, combinedMask);
        m_Mat.copyTo(fgProjDepthMat, combinedMask);
    }
    else
    {
        m_Mat.copyTo(fgProjDepthMat, m_Mask);
    }
    
	MatToPointCloud(fgProjDepthMat, cloud);
}


void DepthFrame::getUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	// Remove player from depth, setting user region depth values to 0 (like measurment errors)
	cv::Mat userFreeProjDepthMat;
	getUserFreeDepthMap(userFreeProjDepthMat);

	MatToPointCloud(userFreeProjDepthMat, cloud);
}


void DepthFrame::getForegroundUserFreePointCloud(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	// Remove player from depth, setting user region depth values to 0 (like measurment errors)
	cv::Mat userFreeProjDepthMat;
	getUserFreeDepthMap(userFreeProjDepthMat);

	cv::Mat fgUserFreeProjDepthMat;
	userFreeProjDepthMat.copyTo(fgUserFreeProjDepthMat, m_Mask);

	MatToPointCloud(fgUserFreeProjDepthMat, cloud);
}

void DepthFrame::setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals)
{
    m_pNormals = pNormals;
}

pcl::PointCloud<pcl::Normal>::Ptr DepthFrame::getNormals()
{
    return m_pNormals;
}

void DepthFrame::getNormals(pcl::PointCloud<pcl::Normal>& normals)
{
    normals = *m_pNormals;
}