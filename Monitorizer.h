//
//  Monitorizer.h
//  remedi2
//
//  Created by Albert Clapés on 24/08/14.
//
//

#ifndef __remedi2__Monitorizer__
#define __remedi2__Monitorizer__

#include "ColorDepthFrame.h"

#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

using namespace std;

class Monitorizer
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointXYZRGB ColorPointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB> ColorPointCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloudPtr;
    typedef pcl::visualization::PCLVisualizer Visualizer;
    typedef pcl::visualization::PCLVisualizer::Ptr VisualizerPtr;
    
public:
    Monitorizer();
    
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    void setMorhologyLevel(int level);
    void setDownsamplingSize(float leafSize);
    void setClusteringIntradistanceFactor(float factor);
    void setMinClusterSize(int minSize);
    
    void detect(vector<vector<PointT> >& positions);
    
    typedef boost::shared_ptr<Monitorizer> Ptr;
    
private:
    void clusterize(PointCloudPtr pCloud, float leafSize, float intraDistFactor, int minSize, vector<PointCloudPtr>& clusters);
    void downsample(PointCloudPtr pCloud, float leafSize, PointCloud& cloudFiltered);
    
    //
    // Attributes
    //
    
    vector<ColorDepthFrame::Ptr> m_InputFrames;
    
    int m_MorphLevel;
    float m_LeafSize;
    float m_ClusterIdF; // Cluster intradistance factor
    float m_MinClusterSize;
};

#endif /* defined(__remedi2__Monitorizer__) */
