#include "TableModeler.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


TableModeler::TableModeler()
: m_bLimitsNegative(false), m_pPlaneCoeffsA(new pcl::ModelCoefficients), m_pPlaneCoeffsB(new pcl::ModelCoefficients)
{
}

TableModeler::TableModeler(const TableModeler& other)
{
    m_pCloudA = other.m_pCloudA;
    m_pCloudB = other.m_pCloudB;
    
	m_LeafSize = other.m_LeafSize;
	m_NormalRadius = other.m_NormalRadius;
	m_SACIters = other.m_SACIters;
	m_SACDistThresh = other.m_SACDistThresh;
    m_bLimitsNegative = other.m_bLimitsNegative;
    m_InteractionBorder = other.m_InteractionBorder;
    m_ConfidenceLevel = other.m_ConfidenceLevel;
    
    m_pPlaneCoeffsA = other.m_pPlaneCoeffsA;
    m_pPlaneCoeffsB = other.m_pPlaneCoeffsB;
    
    m_ytonA = other.m_ytonA;
    m_ytonB = other.m_ytonB;
    
	m_MinA = other.m_MinA;
    m_MaxA = other.m_MaxA;
    m_MinB = other.m_MinB;
    m_MaxB = other.m_MaxB;
    
    m_OffsetA = other.m_OffsetA;
    m_OffsetB = other.m_OffsetB;
    
    m_ytonAInv = other.m_ytonAInv;
    m_ytonBInv = other.m_ytonBInv;
}

TableModeler::~TableModeler()
{
}

void TableModeler::setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB)
{
	m_pCloudA = pCloudA;
	m_pCloudB = pCloudB;
}

void TableModeler::setLeafSize(float leafSize)
{
	m_LeafSize = leafSize;
}

void TableModeler::setNormalRadius(float normalRadius)
{
	m_NormalRadius = normalRadius;
}

void TableModeler::setSACIters(int iters)
{
	m_SACIters = iters;
}

void TableModeler::setSACDistThresh(float distThresh)
{
	m_SACDistThresh = distThresh;
}

void TableModeler::setYOffset(float yOffset)
{
	m_OffsetA = yOffset;
	m_OffsetB = yOffset;
}

void TableModeler::setInteractionBorder(float border)
{
	m_InteractionBorder = border;
}

void TableModeler::setConfidenceLevel(int level)
{
    m_ConfidenceLevel = level;
}

void TableModeler::model()
{
//    std::cout << "Estimating plane in A ..." << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pSceneA (new pcl::PointCloud<pcl::PointXYZ>), m_pPlaneA (new pcl::PointCloud<pcl::PointXYZ>);
    
	if (m_pPlaneCoeffsA->values.empty())
        estimate(m_pCloudA, *m_pPlaneA, *m_pSceneA, *m_pPlaneCoeffsA);
    
    computeTransformation(m_pPlaneCoeffsA, m_ytonA);
    m_ytonAInv = m_ytonA.inverse();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pPlaneTransformedA (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*m_pPlaneA, *m_pPlaneTransformedA, m_ytonA);
    getMinMax3D(m_pPlaneTransformedA, 1, m_ConfidenceLevel, m_MinA, m_MaxA);
    
//    visualizePlaneEstimation(m_pSceneA, m_pPlaneA, m_ytonA, m_MinA, m_MaxA);
    
//    std::cout << "Esitmating plane in B ..." << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pSceneB (new pcl::PointCloud<pcl::PointXYZ>), m_pPlaneB (new pcl::PointCloud<pcl::PointXYZ>);
    
    if (m_pPlaneCoeffsB->values.empty())
        estimate(m_pCloudB, *m_pPlaneB, *m_pSceneB, *m_pPlaneCoeffsB);
    
    computeTransformation(m_pPlaneCoeffsB, m_ytonB);
    m_ytonBInv = m_ytonB.inverse();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pPlaneTransformedB (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*m_pPlaneB, *m_pPlaneTransformedB, m_ytonB);
    getMinMax3D(m_pPlaneTransformedB, 1, m_ConfidenceLevel, m_MinB, m_MaxB);
    
//    visualizePlaneEstimation(m_pSceneA, m_pPlaneA, m_ytonB, m_MinB, m_MaxB);
}

float getPVal(int confidence)
{
    float pval;
    
    if (confidence == 80) pval = 1.28;
    else if (confidence == 85) pval = 1.44;
    else if (confidence == 90) pval = 1.65;
    else if (confidence == 95) pval = 1.96;
    else if (confidence == 99) pval = 2.57;
    else pval = 1.96;
    
    return pval;
}

void TableModeler::getMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, int dim, int confidence, pcl::PointXYZ& min, pcl::PointXYZ& max)
{
    cv::Mat values (pCloud->size(), 3, cv::DataType<float>::type);
    for (int i = 0; i < pCloud->size(); i++)
    {
        values.at<float>(i,0) = pCloud->points[i].x;
        values.at<float>(i,1) = pCloud->points[i].y;
        values.at<float>(i,2) = pCloud->points[i].z;
    }

    for (int i = 0; i < 3; i++)
    {
        if (i == dim)
        {
            cv::Scalar mean, stddev;
            cv::meanStdDev(values.col(dim), mean, stddev);
            
            float pm = getPVal(confidence) * (stddev.val[0] / sqrt(values.rows));

            min.data[i] = mean.val[0] - pm;
            max.data[i] = mean.val[0] + pm;
        }
        else
        {
            double minVal, maxVal;
            cv::minMaxIdx(values.col(i), &minVal, &maxVal);
             
            min.data[i] = minVal;
            max.data[i] = maxVal;
        }
    }
}

void TableModeler::computeTransformation(pcl::ModelCoefficients::Ptr pCoefficients, Eigen::Affine3f& yton)
{
    // Compute a transformation in which a bounding box in a convenient base

    Eigen::Vector3f origin (0, 0, pCoefficients->values[3]/pCoefficients->values[2]);
    Eigen::Vector3f n (pCoefficients->values[0], pCoefficients->values[1], pCoefficients->values[2]);
    //n = -n; // it is upside down
    Eigen::Vector3f u ( 1, 1, (- n.x() - n.y() ) / n.z() );
    Eigen::Vector3f v = n.cross(u);

    pcl::getTransformationFromTwoUnitVectorsAndOrigin(n.normalized(), v.normalized(), -origin, yton);
}

//void TableModeler::transform(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane, Eigen::Affine3f transformation, pcl::PointCloud<pcl::PointXYZ> planeTransformed)
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneT (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*pPlane, *pPlaneT, yton);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneTF (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneF (new pcl::PointCloud<pcl::PointXYZ>);
//
//    // Create the filtering object
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (pPlaneT);
//    sor.setMeanK (100);
//    sor.setStddevMulThresh (1);
//    sor.filter (*pPlaneTF);
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestClusterT (new pcl::PointCloud<pcl::PointXYZ>);
////			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestCluster (new pcl::PointCloud<pcl::PointXYZ>);
//    pBiggestCluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//
//    biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);
//
//    pcl::getMinMax3D(*pBiggestClusterT, min, max);
//    getPointsDimensionCI(*pBiggestClusterT, 2, 0.8, min.y, max.y);
//
//    pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudT (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*pCloud, *pCloudT, yton);
//}

void TableModeler::estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointCloud<pcl::PointXYZ>& planeSegmented, pcl::PointCloud<pcl::PointXYZ>& nonPlaneSegmented, pcl::ModelCoefficients& coefficients)
{
	// downsampling

	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (pCloud);
	sor.setLeafSize (m_LeafSize, m_LeafSize, m_LeafSize);
	sor.filter(*pCloudFiltered);
	
	// normal estimation

	pcl::PointCloud<pcl::Normal>::Ptr pNormalsFiltered (new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(pCloudFiltered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (m_NormalRadius); // neighbors in a sphere of radius X meter
    ne.compute (*pNormalsFiltered);

	// model estimation

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sac (false);
	sac.setOptimizeCoefficients (true); // optional
    sac.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType (pcl::SAC_RANSAC);
    sac.setMaxIterations (m_SACIters);
    sac.setDistanceThreshold (m_SACDistThresh);

    // create filtering object
    
    pcl::ExtractIndices<pcl::PointXYZ> pointExtractor;
    pcl::ExtractIndices<pcl::Normal>   normalExtractor;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pNonPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr   pNonPlaneNormals (new pcl::PointCloud<pcl::Normal>);
    
    pcl::ModelCoefficients::Ptr pCoefficients (new pcl::ModelCoefficients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudFilteredAux (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr pNormalsFilteredAux (new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*pCloudFiltered, *pCloudFilteredAux);
    pcl::copyPointCloud(*pNormalsFiltered, *pNormalsFilteredAux);
    
	bool found = false; // table plane was found
    int numOfInitialPoints = pCloudFiltered->points.size();
	while (!found && pCloudFilteredAux->points.size() > 0.3 * numOfInitialPoints) // Do just one iteration!
    {
        // Segment the largest planar component from the remaining cloud
        sac.setInputCloud(pCloudFilteredAux);
        sac.setInputNormals(pNormalsFilteredAux);
        sac.segment(*inliers, *pCoefficients);

        // Extract the inliers (points in the plane)
        pointExtractor.setInputCloud (pCloudFilteredAux);
        pointExtractor.setIndices (inliers);
        pointExtractor.setNegative (false);
        pointExtractor.filter (*pPlane);
        pointExtractor.setNegative(true);
        pointExtractor.filter(*pNonPlane);
        
        normalExtractor.setInputCloud(pNormalsFilteredAux);
        normalExtractor.setIndices(inliers);
        normalExtractor.setNegative(true);
        normalExtractor.filter(*pNonPlaneNormals);

		if ( (found = isPlaneIncludingOrigin(pPlane)) )
		{
            // Prepare to return the segmented plane
            pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneFiltered (new pcl::PointCloud<pcl::PointXYZ>);
            
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (pPlane);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1);
			sor.filter (*pPlaneFiltered);
            biggestEuclideanCluster(pPlaneFiltered, 2.5f * m_LeafSize, planeSegmented);
            
            coefficients = *pCoefficients;
            
            // Prepare to return the non-plane cloud
            nonPlaneSegmented += *pNonPlane;
            
            return;
        }
        else
        {
            nonPlaneSegmented += *pPlane;
        }
        
        pCloudFilteredAux.swap(pNonPlane);
        pNormalsFilteredAux.swap(pNonPlaneNormals);
        
//			// Compute a transformation in which a bounding box in a convenient base
//
//			Eigen::Vector3f origin (0, 0, coefficients->values[3]/coefficients->values[2]);
//			Eigen::Vector3f n (coefficients->values[0], coefficients->values[1], coefficients->values[2]);
//			//n = -n; // it is upside down
//			Eigen::Vector3f u ( 1, 1, (- n.x() - n.y() ) / n.z() );
//			Eigen::Vector3f v = n.cross(u);
//
//			pcl::getTransformationFromTwoUnitVectorsAndOrigin(n.normalized(), v.normalized(), -origin, yton);
//
//			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneT (new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::transformPointCloud(*pPlane, *pPlaneT, yton);
//
//			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneTF (new pcl::PointCloud<pcl::PointXYZ>);
//			pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneF (new pcl::PointCloud<pcl::PointXYZ>);
//			 
//			// Create the filtering object
//			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//			sor.setInputCloud (pPlaneT);
//			sor.setMeanK (100);
//			sor.setStddevMulThresh (1);
//			sor.filter (*pPlaneTF);
//
//			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestClusterT (new pcl::PointCloud<pcl::PointXYZ>);
////			pcl::PointCloud<pcl::PointXYZ>::Ptr pBiggestCluster (new pcl::PointCloud<pcl::PointXYZ>);
//            pBiggestCluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//
//			biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);
//
//			pcl::getMinMax3D(*pBiggestClusterT, min, max);
//            getPointsDimensionCI(*pBiggestClusterT, 2, 0.8, min.y, max.y);
//
//			pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());
//            pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudT (new pcl::PointCloud<pcl::PointXYZ>);
//            pcl::transformPointCloud(*pCloud, *pCloudT, yton);
//            //pcl::copyPointCloud(*pPlane, plane);
//            
//            // DEBUG
//            // Visualization
//            
////            pcl::visualization::PCLVisualizer pViz(pCloud->header.frame_id);
////            pViz.addCoordinateSystem();
////            //pViz.addPointCloud(pCloudF, "filtered");
////            pViz.addPointCloud(pBiggestCluster, "biggestCluster");
////            pViz.addPointCloud(pBiggestClusterT, "biggestClusterT");
////            pViz.addPointCloud(pCloudT, "pCloudT");
////            pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.61, 0.1, 1.0, "biggestCluster");
////            pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.21, 0.1, 1.0, "biggestClusterT");
////            pViz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 0, 0, "cube");
////            pViz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
////                         min.y, max.y + 2.0,
////                         min.z - m_InteractionBorder, max.z + m_InteractionBorder, 1, 1, 1, "cube2");
////
////            pViz.spin();
//
//			return;
//		}

    }
}

void TableModeler::visualizePlaneEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr pScene, pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane, Eigen::Affine3f transformation, pcl::PointXYZ min, pcl::PointXYZ max)
{
    
    pcl::visualization::PCLVisualizer pViz;
    pViz.addCoordinateSystem();

    pViz.addPointCloud(pScene, "scene");
    pViz.addPointCloud(pPlane, "plane");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pSceneTransformed(new pcl::PointCloud<pcl::PointXYZ>), pPlaneTransformed(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::transformPointCloud(*pScene, *pSceneTransformed, transformation);
    pcl::transformPointCloud(*pPlane, *pPlaneTransformed, transformation);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pPlaneTransformedFiltered(new pcl::PointCloud<pcl::PointXYZ>), pPlaneTransformedRemoved(new pcl::PointCloud<pcl::PointXYZ>);
    
    filter(pPlaneTransformed, min, max, *pPlaneTransformedFiltered, *pPlaneTransformedRemoved);
    
    pViz.addPointCloud(pSceneTransformed, "scene transformed");
    pViz.addPointCloud(pPlaneTransformedFiltered, "plane transformed filtered");
    pViz.addPointCloud(pPlaneTransformedRemoved, "plane transformed removed");

    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "scene");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "plane");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "scene transformed");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "plane transformed filtered");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane transformed filtered");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane transformed removed");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane transformed removed");
    
    pViz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 1, 0, "cube");
    pViz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
                 min.y, max.y + 2.0,
                 min.z - m_InteractionBorder, max.z + m_InteractionBorder, 0, 1, 1, "cube2");

    pViz.spin();
}


bool TableModeler::isPlaneIncludingOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr pPlane)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const pcl::PointXYZ & p =  pPlane->points[i];
        float dist = sqrtf(powf(p.x,2)+powf(p.y,2)+powf(p.z,2));
        
		if ( dist > 0 && dist < 2 * m_LeafSize)
			return true;
	}
    
	return false;
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                   pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	segmentTableTop(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB, pcl::PointCloud<pcl::PointXYZ>& cloudObjsA, pcl::PointCloud<pcl::PointXYZ>& cloudObjsB)
{
	segmentTableTop(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
	segmentTableTop(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
}


void TableModeler::segmentTableTop(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max, float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv, pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAux (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAuxF (new pcl::PointCloud<pcl::PointXYZ>());
    
	// Transformation
	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
	
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr inTableTopRegionCondition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min.x)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max.x)));
    
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y + offset)));
    
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min.z)));
    inTableTopRegionCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max.z)));
    
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem (inTableTopRegionCondition);
    
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
	// De-transformation
    
	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
}

void TableModeler::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, pcl::PointXYZ min, pcl::PointXYZ max, pcl::PointCloud<pcl::PointXYZ>& cloudFiltered, pcl::PointCloud<pcl::PointXYZ>& cloudRemoved)
{
    // build the condition for "within tabletop region"
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min.x)));
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max.x)));
    
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, min.y)));
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y)));
    
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min.z)));
    condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max.z)));
    
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> removal (condition, true); // true = keep indices from filtered points
    removal.setInputCloud (pCloud);
    removal.filter(cloudFiltered);
    
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(pCloud);
    extractor.setIndices(removal.getRemovedIndices());
    extractor.filter(cloudRemoved);
}

void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	segmentInteractionRegion(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
}


void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudA,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudB,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjsA,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjsB)
{
	segmentInteractionRegion(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
	segmentInteractionRegion(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
}


void TableModeler::segmentInteractionRegion(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud,
                                            pcl::PointXYZ min, pcl::PointXYZ max,
                                            float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv,
                                            pcl::PointCloud<pcl::PointXYZ>& cloudObjs)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAux (new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudAuxF (new pcl::PointCloud<pcl::PointXYZ>());

	// Transformation
	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
    
//    pcl::visualization::PCLVisualizer viz ("hola");
//    viz.addCoordinateSystem();
//    int c = 0;
//    
//    viz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 1, 1, "cube");
//    viz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
//                 max.y, max.y + 1.5,
//                 min.z - m_InteractionBorder, max.z + m_InteractionBorder, 1, 0, 0, "cube2");
    
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    
    // build the condition
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr inInteractionRegionRangeCondition (new pcl::ConditionAnd<pcl::PointXYZ> ());
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, min.x - m_InteractionBorder)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, max.x + m_InteractionBorder)));
    
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y + offset + m_InteractionBorder)));
    
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, min.z - m_InteractionBorder)));
    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, max.z + m_InteractionBorder)));
    
    condrem.setCondition(inInteractionRegionRangeCondition);
    
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
    
    pCloudAuxF.swap(pCloudAux);
    
    pcl::ConditionOr<pcl::PointXYZ>::Ptr outTableTopRangeCondition (new pcl::ConditionOr<pcl::PointXYZ> ());
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, min.x)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, max.x)));
    
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, max.y)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, max.y + offset)));
    
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, min.z)));
    outTableTopRangeCondition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, max.z)));
    
    condrem.setCondition(outTableTopRangeCondition);
    condrem.setInputCloud (pCloudAux);
    condrem.setKeepOrganized(true);
    condrem.filter (*pCloudAuxF);
    
	// De-transformation
    
//    cout << "out tabletop filtered" << endl;
//    viz.removeAllPointClouds();
//    viz.addPointCloud (pCloudAuxF, "cloud left");
//    c = 0;
//    while(c++ < 10)
//    {
//        viz.spinOnce(1000);
//    }
    
	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
}

void TableModeler::read(string path, string filename, string extension)
{
    ifstream file;
    string filepath = path + filename + "." + extension;
    file.open(filepath, ios::in);
    if (file.is_open())
    {
        cout << "Reading plane from A ... " << endl;
        
        file >> m_MinA.x >> m_MinA.y >> m_MinA.z;
        file >> m_MaxA.x >> m_MaxA.y >> m_MaxA.z;
        
        m_pPlaneCoeffsA->values.resize(4);
        file >> m_pPlaneCoeffsA->values[0] >> m_pPlaneCoeffsA->values[1] >> m_pPlaneCoeffsA->values[2] >> m_pPlaneCoeffsA->values[3];
        computeTransformation(m_pPlaneCoeffsA, m_ytonA);
        m_ytonAInv = m_ytonA.inverse();
        
        // Debug
        cout << "minA : " << m_MinA << endl;
        cout << "maxA : " << m_MaxA << endl;
        cout << "coefficientsA : (" << m_pPlaneCoeffsA->values[0] << "," << m_pPlaneCoeffsA->values[1] << "," << m_pPlaneCoeffsA->values[2] << "," << m_pPlaneCoeffsA->values[3] << ")" << endl;
        
        
        cout << "Reading plane from B ... " << endl;
        
        file >> m_MinB.x >> m_MinB.y >> m_MinB.z;
        file >> m_MaxB.x >> m_MaxB.y >> m_MaxB.z;
        
        m_pPlaneCoeffsB->values.resize(4);
        file >> m_pPlaneCoeffsB->values[0] >> m_pPlaneCoeffsB->values[1] >> m_pPlaneCoeffsB->values[2] >> m_pPlaneCoeffsB->values[3];
        computeTransformation(m_pPlaneCoeffsB, m_ytonB);
        m_ytonBInv = m_ytonB.inverse();
        
        // Debug
        cout << "minB : " << m_MinB << endl;
        cout << "maxB : " << m_MaxB << endl;
        cout << "coefficientsB : (" << m_pPlaneCoeffsB->values[0] << "," << m_pPlaneCoeffsB->values[1] << "," << m_pPlaneCoeffsB->values[2] << "," << m_pPlaneCoeffsB->values[3] << ")" << endl;
    }
}

void TableModeler::write(string path, string filename, string extension)
{
    ofstream file;
    string filepath = path + filename + "." + extension;
    file.open(filepath, ios::out);
    file << m_MinA.x << " " << m_MinA.y << " " << m_MinA.z << endl;
    file << m_MaxA.x << " " << m_MaxA.y << " " << m_MaxA.z << endl;
    file << m_pPlaneCoeffsA->values[0] << " " << m_pPlaneCoeffsA->values[1] << " " << m_pPlaneCoeffsA->values[2] << " " << m_pPlaneCoeffsA->values[3] << endl;
    
    file << m_MinB.x << " " << m_MinB.y << " " << m_MinB.z << endl;
    file << m_MaxB.x << " " << m_MaxB.y << " " << m_MaxB.z << endl;
    file << m_pPlaneCoeffsB->values[0] << " " << m_pPlaneCoeffsB->values[1] << " " << m_pPlaneCoeffsB->values[2] << " " << m_pPlaneCoeffsB->values[3] << endl;
}

