#pragma once

#include "InteractiveRegisterer.h"
#include "DepthFrame.h"
#include "ColorFrame.h"
#include "ColorDepthFrame.h"

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/correspondence.h>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/common/common.h>

#include <opencv2/opencv.hpp>

using namespace std;

static const float g_Colors[][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
    {1, 0, 1},
    {1, 1, 0},
    {0, 1, 1},
    {.5, 0, 0},
    {0, .5, 0},
    {0, 0, .5},
    {1, 0, .5},
    {1, .5, 0},
    {1, .5, .5},
    {0, 1, .5},
    {.5, 1, 0},
    {.5, 0, .5},
    {0, .5, .5}
};

//////////////////////////////////////////////

class InteractiveRegisterer
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
	InteractiveRegisterer();
    InteractiveRegisterer(const InteractiveRegisterer& rhs);
    
    InteractiveRegisterer& operator=(const InteractiveRegisterer& rhs);
    
    // Set the number of correspondences
	void setNumPoints(int numOfPoints);
    
    // Set the frames used to establish the correspondences
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    
    // Set the distribution of the viewports in the visualizer
    void setVisualizerParameters(int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius);
    
    // Manually interact with a visualizer to place the markers that serve as correspondences
    void setCorrespondencesManually();
    void saveCorrespondences(string path, string extension);
    void loadCorrespondences(string path, string extension);
    
    // Estimate the orthogonal transformations in the n-1 views respect to the 0-th view
    void computeTransformations();
    
    void registrate(vector<ColorDepthFrame::Ptr> pUnregFrames, vector<PointCloudPtr>& pRegClouds);
    void registrate(vector<PointCloudPtr> pUnregClouds, vector<PointCloudPtr>& pRegClouds);

    
    // ok

//    void translate(const PointCloudPtr, PointT, PointCloud&);
//    void translate(const PointCloudPtr, Eigen::Vector4f, PointCloud&);
    

    
//
////    void align(DepthFrame&, DepthFrame&);
//
//    void computeTransformation();
//    void computeFineTransformation();
//    
//	bool loadTransformation(string filePath);
//	void saveTransformation(string filePath);
//    
//    void registration(DepthFrame::Ptr, DepthFrame::Ptr,
//                      PointCloud&, PointCloud&,
//                      bool backgroundPoints = true, bool userPoints = true);
//    void registration(ColorDepthFrame::Ptr, ColorDepthFrame::Ptr,
//                      PointCloud&, PointCloud&,
//                      bool backgroundPoints = true, bool userPoints = true);
//    
//    PointT registration(PointT point, int viewpoint);
//    void registration(PointT point, PointT& regPoint, int viewpoint);
//    void registration(PointT pointA, PointT pointB, PointT& regPointA, PointT& regPointB);
//	void registration(PointCloudPtr pCloudA, PointCloudPtr pCloudB, PointCloud& regCloudA, PointCloud& regCloudB);
//    void registration(PointCloudPtr pCloud, PointCloud& regCloud, int viewpoint);
//    void registration(vector<PointCloudPtr> pCloudsA, vector<PointCloudPtr> pCloudsB, vector<PointCloudPtr>& pRegCloudsA, vector<PointCloudPtr>& pRegCloudsB);
//    
//    PointT deregistration(PointT regPoint, int viewpoint);
//    void deregistration(PointT regPoint, PointT& point, int viewpoint);
//    void deregistration(PointT regPointA, PointT regPointB, PointT& pointA, PointT& pointB);
//    void deregistration(PointCloudPtr pRegCloudA, PointCloudPtr pRegCloudB, PointCloud& cloudA, PointCloud& cloudB);
//    void deregistration(PointCloudPtr pRegCloud, PointCloud& cloud, int viewpoint);
//    void deregistration(vector<PointCloudPtr> pRegCloudsA, vector<PointCloudPtr> pRegCloudsB, vector<PointCloudPtr>& pCloudsA, vector<PointCloudPtr>& pCloudsB);
//
//    pair<PointCloudPtr,PointCloudPtr> getRegisteredClouds();
//    
//	void visualizeRegistration(PointCloudPtr, PointCloudPtr);
//	void visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr, 
//		PointCloudPtr, PointCloudPtr);
//	void visualizeRegistration(DepthFrame::Ptr, DepthFrame::Ptr);
//    
//    PointT getLeftRefPoint();
//    PointT getRightRefPoint();
    
    typedef boost::shared_ptr<InteractiveRegisterer> Ptr;
    
private:
    // Callback function to deal with keyboard presses in visualizer
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* ptr);
    // Callback function to deal with mouse presses in visualizer
    void mouseCallback(const pcl::visualization::MouseEvent& event, void* ptr);
    // Callback function to deal with point picking actions in visualizer (shift+mouse press)
    void ppCallback(const pcl::visualization::PointPickingEvent& event, void* ptr);
    
    void setDefaultCamera(VisualizerPtr pViz, int vid);
    
    void getTransformation(const PointCloudPtr pSrcMarkersCloud, const PointCloudPtr pTgtMarkersCloud, Eigen::Matrix4f& T);
    
////    void align (const PointCloudPtr, const PointCloudPtr, PointCloudPtr, PointCloudPtr, PointCloudPtr, PointCloudPtr);
    
    // Members
    
    VisualizerPtr m_pViz;
    vector<int> m_VIDs; // viewport identifiers
    int m_WndHeight, m_WndWidth;
    int m_Vp, m_Hp;
    float m_CameraDistance;
    float m_MarkerRadius;
    
    int m_Viewport; // current viewport
    bool m_bMark;   // mark keyboard button is down
    
    vector<ColorDepthFrame::Ptr> m_pFrames;

    vector<ColorPointCloudPtr> m_pClouds;
    vector<PointCloudPtr> m_pMarkers;
    
    // Re-position the points
    bool m_bReset; //reallocate_points_;
    // Number of points to compute the transformation
    int m_NumOfPoints; // number of points //int num_points_;
    int m_Pendents;
    
//    bool m_bTranslate, m_bAlign; //must_translate_, must_align_;
    
    pcl::CorrespondencesPtr m_pCorrespondences; //corresps_;

    vector<Eigen::Vector4f> m_vMarkers;
    vector<Eigen::Matrix4f> m_Transformations;
    vector<Eigen::Matrix4f> m_ITransformations;
    
//	Eigen::Vector4f m_tLeft, m_tRight;
//	Eigen::Matrix4f m_Transformation, m_InverseTransformation;
};