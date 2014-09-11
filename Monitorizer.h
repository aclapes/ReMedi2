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
#include "Cloudject.hpp"
#include "CloudjectModel.hpp"

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
    typedef LFCloudject<PointT, pcl::FPFHSignature33> Cloudject;
    typedef LFCloudject<PointT, pcl::FPFHSignature33>::Ptr CloudjectPtr;
    typedef LFCloudjectModel<PointT, pcl::FPFHSignature33> CloudjectModel;
    typedef LFCloudjectModel<PointT, pcl::FPFHSignature33>::Ptr CloudjectModelPtr;
    
public:
    Monitorizer();
    
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    void setMorhologyLevel(int level);
    void setDownsamplingSize(float leafSize);
    void setClusteringIntradistanceFactor(float factor);
    void setMinClusterSize(int minSize);
    void setInterviewCorrepondenceDistance(float d);
    
    void detect(vector<vector<PointCloudPtr> >& detections);
    void detect(vector<vector<PointT> >& positions);
    void getDetectionsPositions(vector<vector<PointCloudPtr> > detections, vector<vector<PointT> >& positions);
    
    void recognize(vector<CloudjectPtr>& recognitions);
    void recognize(vector<vector<vector<pcl::PointXYZ> > >& positions);
    
    typedef boost::shared_ptr<Monitorizer> Ptr;
    
private:
    void clusterize(PointCloudPtr pCloud, float leafSize, float intraDistFactor, int minSize, vector<PointCloudPtr>& clusters);
    void downsample(PointCloudPtr pCloud, float leafSize, PointCloud& cloudFiltered);
    void cloudjectify(vector<vector<PointCloudPtr> > detections, vector<CloudjectPtr>& cloudjects);
    void findCorrespondences(vector<vector<PointCloudPtr> > detections, float tol, vector<vector<pair<int,PointCloudPtr> > >& correspondences);
    void findNextCorrespondence(vector<vector<PointT> >& detections, vector<vector<bool> >& assignations, int v, float tol, vector<pair<pair<int,int>,PointT> >& chain);

    
    //
    // Attributes
    //
    
    vector<ColorDepthFrame::Ptr> m_InputFrames;
    
    int m_MorphLevel;
    float m_LeafSize;
    float m_ClusterIdF; // Cluster intradistance factor
    float m_MinClusterSize;
    float m_CorrespenceDist;
    
    vector<CloudjectModelPtr> m_CloudjectModels;
};

#endif /* defined(__remedi2__Monitorizer__) */
