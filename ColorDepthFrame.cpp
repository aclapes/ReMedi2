//
//  ColorDepthFrame.cpp
//  remedi
//
//  Created by Albert Clapés on 11/07/14.
//
//

#include "ColorDepthFrame.h"

ColorDepthFrame::ColorDepthFrame()
{
    
}

ColorDepthFrame::ColorDepthFrame(cv::Mat colorMat, cv::Mat depthMat)
: ColorFrame(colorMat), DepthFrame(depthMat), m_pColoredCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    // Create a point cloud
    MatToColoredPointCloud(DepthFrame::get(), ColorFrame::get(), *m_pColoredCloud);
}

ColorDepthFrame::ColorDepthFrame(cv::Mat colorMat, cv::Mat depthMat, cv::Mat colorMask, cv::Mat depthMask)
: ColorFrame(colorMat, colorMask), DepthFrame(depthMat, depthMask), m_pColoredCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    // Create a point cloud
    MatToColoredPointCloud(DepthFrame::get(), ColorFrame::get(), *m_pColoredCloud);
}

ColorDepthFrame::ColorDepthFrame(const ColorDepthFrame& rhs)
: ColorFrame(rhs), DepthFrame(rhs)
{
    *this = rhs;
}

ColorDepthFrame& ColorDepthFrame::operator=(const ColorDepthFrame& rhs)
{
    if (this != &rhs)
    {
        ColorFrame::operator=(rhs);
        DepthFrame::operator=(rhs);
        
        m_pColoredCloud = rhs.m_pColoredCloud;
    }
    
    return *this;
}

void ColorDepthFrame::set(cv::Mat color, cv::Mat depth)
{
    ColorFrame::set(color);
    DepthFrame::set(depth);
}

void ColorDepthFrame::setColor(cv::Mat color)
{
    ColorFrame::set(color);
}

void ColorDepthFrame::setDepth(cv::Mat depth)
{
    DepthFrame::set(depth);
}

void ColorDepthFrame::setColor(cv::Mat color, cv::Mat mask)
{
    ColorFrame::set(color, mask);
}

void ColorDepthFrame::setDepth(cv::Mat depth, cv::Mat mask)
{
    DepthFrame::set(depth, mask);
}

void ColorDepthFrame::setColorMask(cv::Mat mask)
{
    ColorFrame::setMask(mask);
}

void ColorDepthFrame::setDepthMask(cv::Mat mask)
{
    DepthFrame::setMask(mask);
}

void ColorDepthFrame::get(cv::Mat& color, cv::Mat& depth)
{
    ColorFrame::get(color);
    DepthFrame::get(depth);
}

cv::Mat ColorDepthFrame::getColor()
{
    return ColorFrame::get();
}

cv::Mat ColorDepthFrame::getDepth()
{
    return DepthFrame::get();
}

void ColorDepthFrame::getColor(cv::Mat& color)
{
    ColorFrame::get(color);
}

void ColorDepthFrame::getDepth(cv::Mat& depth)
{
    DepthFrame::get(depth);
}

cv::Mat ColorDepthFrame::getColorMask()
{
    return ColorFrame::getMask();
}

cv::Mat ColorDepthFrame::getDepthMask()
{
    return DepthFrame::getMask();
}

void ColorDepthFrame::getColorMask(cv::Mat& colorMask)
{
    ColorFrame::getMask(colorMask);
}

void ColorDepthFrame::getDepthMask(cv::Mat& depthMask)
{
    DepthFrame::getMask(depthMask);
}

void ColorDepthFrame::getColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud)
{
    coloredCloud = *m_pColoredCloud;
}

void ColorDepthFrame::setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals)
{
    DepthFrame::setNormals(pNormals);
}

void ColorDepthFrame::getRegisteredColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud)
{
    pcl::transformPointCloud(*m_pColoredCloud, coloredCloud, m_T);
}

void ColorDepthFrame::getRegisteredAndReferencedColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud)
{
    // Register the reference point
    pcl::PointXYZ regReferencePoint = getRegisteredReferencePoint();
    
    // Build a translation matrix to the registered reference the cloud after its own registration
    Eigen::Matrix4f E = Eigen::Matrix4f::Identity();
    E.col(3) = regReferencePoint.getVector4fMap(); // set translation column
    
    // Apply registration first and then referenciation (right to left order in matrix product)
    pcl::transformPointCloud(*m_pColoredCloud, coloredCloud, E.inverse() * m_T);
}