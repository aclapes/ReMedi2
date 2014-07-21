#pragma once

#include "DepthFrame.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

using namespace std;

class TableModeler
{
public:
	TableModeler();
    TableModeler(const TableModeler& other);
	~TableModeler();

	void setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB);

	void setLeafSize(float);
	void setNormalRadius(float);
	void setSACIters(int);
	void setSACDistThresh(float);
	void setYOffset(float);
    void setInteractionBorder(float);
    void setConfidenceLevel(int level);
    
	void model();

    void setInverseSegmentation();
    
	void segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&);
	void segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);
    
    void segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&);
	void segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>&, pcl::PointCloud<pcl::PointXYZ>&);
    
    void read(string path, string name, string extension);
    void write(string path, string name, string extension);
    
    void visualizePlaneEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr pScene, pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane, Eigen::Affine3f transformation, pcl::PointXYZ min, pcl::PointXYZ max);
    
    typedef boost::shared_ptr<TableModeler> Ptr;
    
private:

	void estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pScene, pcl::PointCloud<pcl::PointXYZ>& plane, pcl::PointCloud<pcl::PointXYZ>& nonplane, pcl::ModelCoefficients& coeffs);
    bool isPlaneIncludingOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane);
    void computeTransformation(pcl::ModelCoefficients::Ptr pCoefficients, Eigen::Affine3f& yton);
//    void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane, Eigen::Affine3f transformation, pcl::PointCloud<pcl::PointXYZ> planeTransformed);

    void getMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane, int dim, int confidence, pcl::PointXYZ& min, pcl::PointXYZ& max);

	void segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                         pcl::PointXYZ, pcl::PointXYZ, float offset,
                         Eigen::Affine3f, Eigen::Affine3f,
                         pcl::PointCloud<pcl::PointXYZ>&);
    
    void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointCloud<pcl::PointXYZ>& cloudFiltered, pcl::PointCloud<pcl::PointXYZ>& cloudRemoved);
    
    void segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr,
                                  pcl::PointXYZ, pcl::PointXYZ, float offset,
                                  Eigen::Affine3f, Eigen::Affine3f,
                                  pcl::PointCloud<pcl::PointXYZ>&);

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloudA, m_pCloudB;
//    pcl::PointXYZ m_originA, m_originB;
    
	float m_LeafSize;
	float m_NormalRadius;
	float m_SACIters;
	float m_SACDistThresh;
    float m_ConfidenceLevel;
    
    pcl::ModelCoefficients::Ptr m_pPlaneCoeffsA, m_pPlaneCoeffsB;

	pcl::PointXYZ m_MinA, m_MaxA, m_MinB, m_MaxB;
	float m_OffsetA, m_OffsetB; // sum to the axis corresponding to plane's normal (probably the y dimension)
	Eigen::Affine3f m_ytonA, m_ytonB; // y axis to n plane normal vector transformtion
    Eigen::Affine3f m_ytonAInv, m_ytonBInv; // y axis to n plane normal vector transformtion

    float m_InteractionBorder;
    bool m_bLimitsNegative;
};