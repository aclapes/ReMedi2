//
//  ObjectDetector.h
//  remedi2
//
//  Created by Albert Clapés on 24/08/14.
//
//

#ifndef __remedi2__ObjectDetector__
#define __remedi2__ObjectDetector__

#include "ColorDepthFrame.h"
#include "ObjectModel.hpp"
#include "Cloudject.hpp"
#include "CloudjectModel.hpp"

#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

using namespace std;

class ObjectDetector
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointXYZRGB ColorPointT;
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB> ColorPointCloud;
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloudPtr;
    typedef pcl::visualization::PCLVisualizer Visualizer;
    typedef pcl::visualization::PCLVisualizer::Ptr VisualizerPtr;
    typedef pcl::FPFHSignature33 FPFHSignatureT;
    typedef pcl::PFHRGBSignature250 PFHRGBSignatureT;
    
public:
    ObjectDetector();
    ObjectDetector(const ObjectDetector& rhs);
    ObjectDetector& operator=(const ObjectDetector& rhs);
    
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    void setActorMasks(vector<cv::Mat> masks);
    void setInteractionMasks(vector<cv::Mat> masks);
    
    void setDownsamplingSize(float leafSize);
    void setClusteringIntradistanceFactor(float factor);
    void setMinClusterSize(float minSize);
    void setInterviewCorrepondenceDistance(float d);
    
    void setRegistration(bool registration);
    
    void detect();
    void detect(vector<vector<ColorPointCloudPtr> >& detections);
    
    void getDetectionPositions(vector<vector<PointT> >& positions);
//    void getDetectionCorrespondences(vector<vector<pair<int,PointT> > >& positions, bool bMakeCorrespondences = true);
    void getDetectionCorrespondences(vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences, bool bMakeCorrespondences = true);
        
    typedef boost::shared_ptr<ObjectDetector> Ptr;
    
private:
    void clusterize(ColorPointCloudPtr pCloud, float leafSize, float intraDistFactor, float minSize, std::vector<ColorPointCloudPtr>& clusters);
    void cclusterize(ColorPointCloudPtr pCloud, float leafSize, float intraDistFactor, float minSize, std::vector<ColorPointCloudPtr>& clusters);
    
    void downsample(ColorPointCloudPtr pCloud, float leafSize, ColorPointCloud& cloudFiltered);
    
    void findCorrespondences(vector<vector<ColorPointCloudPtr> > detections, float tol, vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences);
    void _findCorrespondences(vector<vector<PointT> > positions, float tol, vector<vector<pair<pair<int,int>,PointT> > >&  correspondences);
    void findNextCorrespondence(vector<vector<PointT> >& detections, vector<vector<bool> >& assignations, int v, float tol, vector<pair<pair<int,int>,PointT> >& chain);
    
    void _getDetectionPositions(vector<vector<ColorPointCloudPtr> > detections, bool bRegistrate, vector<vector<PointT> >& positions);
    
    void merge(vector<ColorPointCloudPtr> detections, vector<ColorPointCloudPtr>& mergeds);
    
    void subtractInteractionFromActors(std::vector<ColorPointCloudPtr> tabletopRegionClusters, ColorPointCloudPtr interactionCloud, std::vector<int>& indices, float leafSize);
    bool isInteractive(ColorPointCloudPtr tabletopRegionCluster, ColorPointCloudPtr interactionCloud, float leafSize);
    
//    bool enforceCurvatureOrIntensitySimilarity (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance);

    //
    // Attributes
    //
    
    vector<ColorDepthFrame::Ptr> m_InputFrames;
    vector<cv::Mat> m_ActorMasks;
    vector<cv::Mat> m_InteractionMasks;
    
    float m_LeafSize;
    float m_ClusterIdF; // Cluster intradistance factor
    float m_MinClusterSize;
    float m_CorrespenceDist;
    
    bool m_bRegistration;
    
    float m_ColorDist;
    float m_NormalDotThresh; // should range in [0, pi/2]
    
    vector<vector<ColorPointCloudPtr> > m_Detections;
};

#endif /* defined(__remedi2__ObjectDetector__) */
