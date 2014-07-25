//
//  ColorDepthFrame.h
//  remedi
//
//  Created by Albert Clapés on 11/07/14.
//
//

#ifndef __remedi__ColorDepthFrame__
#define __remedi__ColorDepthFrame__

#include "ColorFrame.h"
#include "DepthFrame.h"

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <iostream>

using namespace std;

class ColorDepthFrame : public ColorFrame, public DepthFrame
{
public:
    ColorDepthFrame();
    ColorDepthFrame(cv::Mat colorMat, cv::Mat depthMat);
    ColorDepthFrame(cv::Mat colorMat, cv::Mat depthMat, cv::Mat colorMask, cv::Mat depthMask);
    ColorDepthFrame(const ColorDepthFrame& rhs);
   
    ColorDepthFrame& operator=(const ColorDepthFrame& other);
    
    void set(cv::Mat color, cv::Mat depth);
    void setColor(cv::Mat color);
    void setDepth(cv::Mat depth);
    void setColor(cv::Mat color, cv::Mat mask);
    void setDepth(cv::Mat depth, cv::Mat mask);
    void setColorMask(cv::Mat mask);
    void setDepthMask(cv::Mat mask);
    
    void get(cv::Mat& color, cv::Mat& depth);
    cv::Mat getColor();
    cv::Mat getDepth();
    void getColor(cv::Mat& color);
    void getDepth(cv::Mat& depth);
    cv::Mat getColorMask();
    cv::Mat getDepthMask();
    void getColorMask(cv::Mat& colorMask);
    void getDepthMask(cv::Mat& depthMask);
    
    void getColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud);
    void setNormals(pcl::PointCloud<pcl::Normal>::Ptr pNormals);
    
    void getRegisteredColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud);
    void getRegisteredAndReferencedColoredPointCloud(pcl::PointCloud<pcl::PointXYZRGB>& coloredCloud);
    
    typedef boost::shared_ptr<ColorDepthFrame> Ptr;
    
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pColoredCloud;
};

#endif /* defined(__remedi__ColorDepthFrame__) */
