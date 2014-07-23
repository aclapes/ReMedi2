#include "TableModeler.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


TableModeler::TableModeler()
//: m_bLimitsNegative(false), m_pPlaneCoeffsA(new pcl::ModelCoefficients), m_pPlaneCoeffsB(new pcl::ModelCoefficients)
{
}

TableModeler::TableModeler(const TableModeler& rhs)
{
    *this = rhs;
}

TableModeler& TableModeler::operator=(const TableModeler& rhs)
{
    if (this != &rhs)
    {
        //    m_pCloudA = other.m_pCloudA;
        //    m_pCloudB = other.m_pCloudB;
        
        m_LeafSize = rhs.m_LeafSize;
        m_NormalRadius = rhs.m_NormalRadius;
        m_SACIters = rhs.m_SACIters;
        m_SACDistThresh = rhs.m_SACDistThresh;
        m_YOffset = rhs.m_YOffset;
        m_InteractionBorder = rhs.m_InteractionBorder;
        m_ConfidenceLevel = rhs.m_ConfidenceLevel;
        
        //    m_bLimitsNegative = other.m_bLimitsNegative;
        
        //    m_pPlaneCoeffsA = other.m_pPlaneCoeffsA;
        //    m_pPlaneCoeffsB = other.m_pPlaneCoeffsB;
        //
        //    m_ytonA = other.m_ytonA;
        //    m_ytonB = other.m_ytonB;
        //
        //	m_MinA = other.m_MinA;
        //    m_MaxA = other.m_MaxA;
        //    m_MinB = other.m_MinB;
        //    m_MaxB = other.m_MaxB;
        //
        //    m_OffsetA = other.m_OffsetA;
        //    m_OffsetB = other.m_OffsetB;
        //    
        //    m_ytonAInv = other.m_ytonAInv;
        //    m_ytonBInv = other.m_ytonBInv;
    }
    
    return *this;
}

void TableModeler::setInputFrames(vector<DepthFrame::Ptr> pFrames)
{
	m_pFrames = pFrames;
}

void TableModeler::setInputFrames(vector<ColorDepthFrame::Ptr> pFrames)
{
	for (int v = 0; v < pFrames.size(); v++)
        m_pFrames.push_back(pFrames[v]); // implicit cast
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
	m_YOffset = yOffset;
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
    for (int v = 0; v < m_pFrames.size(); v++)
    {
        PointCloudPtr pCloud (new PointCloud);
        m_pFrames[v]->getPointCloud(*pCloud);
        
        // Estimate the table plane (using RANSAC internally and normals' orientation information)
        
        PointCloudPtr pTable (new PointCloud);
        PointCloudPtr pNonTable (new PointCloud);
        pcl::ModelCoefficients::Ptr pTableCoefficients (new pcl::ModelCoefficients);
        
        estimate(pCloud, *pTable, *pNonTable, *pTableCoefficients);
        
        // Find the transform: table's normal to (0,0,0) and its direction rotated to XY
        
        Eigen::Affine3f T, TI;
        getTransformation(pTableCoefficients, T);
        TI = T.inverse();
        
        // Transform the plane
        
        PointCloudPtr pTableTransformed (new PointCloud);
        pcl::transformPointCloud(*pTable, *pTableTransformed, T);
        
        // Get the table's 3D surrounding bounding box
        
        Point min, max;
        getMinMax3D(pTableTransformed, 1, m_ConfidenceLevel, min, max);
        
        // dbg <--
        visualizePlaneEstimation(pCloud, pTable, T, min, max);
        // dbg -->
    }
}

//void TableModeler::transform(PointCloudPtr pPlane, Eigen::Affine3f transformation, PointCloud planeTransformed)
//{
//    PointCloudPtr pPlaneT (new PointCloud);
//    pcl::transformPointCloud(*pPlane, *pPlaneT, yton);
//
//    PointCloudPtr pPlaneTF (new PointCloud);
//    PointCloudPtr pPlaneF (new PointCloud);
//
//    // Create the filtering object
//    pcl::StatisticalOutlierRemoval<Point> sor;
//    sor.setInputCloud (pPlaneT);
//    sor.setMeanK (100);
//    sor.setStddevMulThresh (1);
//    sor.filter (*pPlaneTF);
//
//    PointCloudPtr pBiggestClusterT (new PointCloud);
////			PointCloudPtr pBiggestCluster (new PointCloud);
//    pBiggestCluster = PointCloudPtr(new PointCloud);
//
//    biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);
//
//    pcl::getMinMax3D(*pBiggestClusterT, min, max);
//    getPointsDimensionCI(*pBiggestClusterT, 2, 0.8, min.y, max.y);
//
//    pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());
//    PointCloudPtr pCloudT (new PointCloud);
//    pcl::transformPointCloud(*pCloud, *pCloudT, yton);
//}

void TableModeler::estimate(PointCloudPtr pCloud, PointCloud& planeSegmented, PointCloud& nonPlaneSegmented, pcl::ModelCoefficients& coefficients)
{
	// downsampling

	PointCloudPtr pCloudFiltered (new PointCloud);
    
	pcl::ApproximateVoxelGrid<Point> sor;
	sor.setInputCloud (pCloud);
	sor.setLeafSize (m_LeafSize, m_LeafSize, m_LeafSize);
	sor.filter(*pCloudFiltered);
	
	// normal estimation

	pcl::PointCloud<pcl::Normal>::Ptr pNormalsFiltered (new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<Point, pcl::Normal> ne;
    ne.setInputCloud(pCloudFiltered);
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (m_NormalRadius); // neighbors in a sphere of radius X meter
    ne.compute (*pNormalsFiltered);

	// model estimation

    pcl::SACSegmentationFromNormals<Point, pcl::Normal> sac (false);
	sac.setOptimizeCoefficients (true); // optional
    sac.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType (pcl::SAC_RANSAC);
    sac.setMaxIterations (m_SACIters);
    sac.setDistanceThreshold (m_SACDistThresh);

    // create filtering object
    
    pcl::ExtractIndices<Point> pointExtractor;
    pcl::ExtractIndices<pcl::Normal>   normalExtractor;

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    PointCloudPtr pPlane (new PointCloud);
    PointCloudPtr pNonPlane (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr   pNonPlaneNormals (new pcl::PointCloud<pcl::Normal>);
    
    pcl::ModelCoefficients::Ptr pCoefficients (new pcl::ModelCoefficients);

    PointCloudPtr pCloudFilteredAux (new PointCloud);
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

		if ( (found = isCloudIncludingOrigin(pPlane)) )
		{
            // Prepare to return the segmented plane
            PointCloudPtr pPlaneFiltered (new PointCloud);
            
            pcl::StatisticalOutlierRemoval<Point> sor;
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
//			PointCloudPtr pPlaneT (new PointCloud);
//			pcl::transformPointCloud(*pPlane, *pPlaneT, yton);
//
//			PointCloudPtr pPlaneTF (new PointCloud);
//			PointCloudPtr pPlaneF (new PointCloud);
//			 
//			// Create the filtering object
//			pcl::StatisticalOutlierRemoval<Point> sor;
//			sor.setInputCloud (pPlaneT);
//			sor.setMeanK (100);
//			sor.setStddevMulThresh (1);
//			sor.filter (*pPlaneTF);
//
//			PointCloudPtr pBiggestClusterT (new PointCloud);
////			PointCloudPtr pBiggestCluster (new PointCloud);
//            pBiggestCluster = PointCloudPtr(new PointCloud);
//
//			biggestEuclideanCluster(pPlaneTF, 2.5 * m_LeafSize, *pBiggestClusterT);
//
//			pcl::getMinMax3D(*pBiggestClusterT, min, max);
//            getPointsDimensionCI(*pBiggestClusterT, 2, 0.8, min.y, max.y);
//
//			pcl::transformPointCloud(*pBiggestClusterT, *pBiggestCluster, yton.inverse());
//            PointCloudPtr pCloudT (new PointCloud);
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

/** \brief Get the orthogonal transformation of a plane so as translate it to the (0,0,0) and to align its normal to one of the coordinate system axis
 *  \param pPlaneCoefficients The coefficients of the plane (normal's components and bias)
 *  \param T The 3D affine transformation itself
 */
void TableModeler::getTransformation(pcl::ModelCoefficients::Ptr pCoefficients, Eigen::Affine3f& T)
{
    Eigen::Vector3f origin (0, 0, pCoefficients->values[3]/pCoefficients->values[2]);
    Eigen::Vector3f n (pCoefficients->values[0], pCoefficients->values[1], pCoefficients->values[2]);
    //n = -n; // if it is upside down, but it is not now :D
    Eigen::Vector3f u ( 1, 1, (- n.x() - n.y() ) / n.z() );
    Eigen::Vector3f v = n.cross(u);
    
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(n.normalized(), v.normalized(), -origin, T);
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

/** \brief Get the 3D bounding box containing a plane
 *  \param pPlane A plane
 *  \param d Dimension in which the plane is constant
 *  \param confidence Confidence level to discard outliers in the d dimension
 *  \param min Minimum value of the 3D bounding box
 *  \param max Maximum value of the 3D bounding box
 */
void TableModeler::getMinMax3D(PointCloudPtr pCloud, int d, int confidence, Point& min, Point& max)
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
        if (i == d)
        {
            cv::Scalar mean, stddev;
            cv::meanStdDev(values.col(d), mean, stddev);
            
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

/** \brief Determines wheter or not a cloud includes the origin, i.e. the (0,0,0) point (or a very near one)
 *  \param pCloud A cloud
 *  \return Includes the origin
 */
bool TableModeler::isCloudIncludingOrigin(PointCloudPtr pPlane)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const Point & p =  pPlane->points[i];
        float dist = sqrtf(powf(p.x,2)+powf(p.y,2)+powf(p.z,2));
        
		if ( dist > 0 && dist < 2 * m_LeafSize)
			return true;
	}
    
	return false;
}

/** \brief Filter the points given a 3D bounding box.
 *  \param pCloud A cloud to filter
 *  \param min The minimum of the box
 *  \param max The maximum of the box
 *  \param cloudFiltered Points of the cloud kept (insiders)
 *  \param cloudRemoved Points of the cloud removed (outsiders)
 */
void TableModeler::filter(PointCloudPtr pCloud, Point min, Point max, PointCloud& cloudFiltered, PointCloud& cloudRemoved)
{
    // build the condition for "within tabletop region"
    pcl::ConditionAnd<Point>::Ptr condition (new pcl::ConditionAnd<Point> ());
    
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::GT, min.x)));
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::LT, max.x)));
    
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::GT, min.y)));
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::LT, max.y)));
    
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::GT, min.z)));
    condition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::LT, max.z)));
    
    // build the filter
    pcl::ConditionalRemoval<Point> removal (condition, true); // true = keep indices from filtered points
    removal.setInputCloud (pCloud);
    removal.filter(cloudFiltered);
    
    pcl::ExtractIndices<Point> extractor;
    extractor.setInputCloud(pCloud);
    extractor.setIndices(removal.getRemovedIndices());
    extractor.filter(cloudRemoved);
}

void TableModeler::visualizePlaneEstimation(PointCloudPtr pScene, PointCloudPtr pTable, Eigen::Affine3f T, Point min, Point max)
{
    pcl::visualization::PCLVisualizer pViz;
    pViz.addCoordinateSystem();

    pViz.addPointCloud(pScene, "scene");
    pViz.addPointCloud(pTable, "plane");
    
    PointCloudPtr pSceneTransformed(new PointCloud);
    PointCloudPtr pTableTransformed(new PointCloud);
    
    pcl::transformPointCloud(*pScene, *pSceneTransformed, T);
    pcl::transformPointCloud(*pTable, *pTableTransformed, T);
    
    PointCloudPtr pTableTransformedFiltered(new PointCloud);
    PointCloudPtr pTableTransformedRemoved(new PointCloud);
    
    filter(pTableTransformed, min, max, *pTableTransformedFiltered, *pTableTransformedRemoved);
    
    pViz.addPointCloud(pSceneTransformed, "scene transformed");
    pViz.addPointCloud(pTableTransformedFiltered, "plane transformed filtered");
    pViz.addPointCloud(pTableTransformedRemoved, "plane transformed removed");

    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "scene");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "plane");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "scene transformed");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "plane transformed filtered");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane transformed filtered");
    
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane transformed removed");
    pViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane transformed removed");
    
    pViz.addCube(min.x, max.x, min.y, max.y + m_YOffset, min.z, max.z, 1, 1, 0, "cube");
    pViz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
                 min.y, max.y + 2.0,
                 min.z - m_InteractionBorder, max.z + m_InteractionBorder, 0, 1, 1, "cube2");

    pViz.spin();
}

//void TableModeler::segmentTableTop(PointCloudPtr pCloud,
//                                   PointCloud& cloudObjs)
//{
//	segmentTableTop(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
//}
//
//
//void TableModeler::segmentTableTop(PointCloudPtr pCloudA, PointCloudPtr pCloudB, PointCloud& cloudObjsA, PointCloud& cloudObjsB)
//{
//	segmentTableTop(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
//	segmentTableTop(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
//}
//
//
//void TableModeler::segmentTableTop(PointCloudPtr pCloud, Point min, Point max, float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv, PointCloud& cloudObjs)
//{
//	PointCloudPtr pCloudAux (new PointCloud());
//	PointCloudPtr pCloudAuxF (new PointCloud());
//    
//	// Transformation
//	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
//	
//    // build the condition
//    pcl::ConditionAnd<Point>::Ptr inTableTopRegionCondition (new pcl::ConditionAnd<Point> ());
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::GT, min.x)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::LT, max.x)));
//    
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::GT, max.y)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::LT, max.y + offset)));
//    
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::GT, min.z)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::LT, max.z)));
//    
//    // build the filter
//    pcl::ConditionalRemoval<Point> condrem (inTableTopRegionCondition);
//    
//    condrem.setInputCloud (pCloudAux);
//    condrem.setKeepOrganized(true);
//    condrem.filter (*pCloudAuxF);
//    
//	// De-transformation
//    
//	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
//}
//
//
//
//void TableModeler::segmentInteractionRegion(PointCloudPtr pCloud,
//                                            PointCloud& cloudObjs)
//{
//	segmentInteractionRegion(pCloud, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjs);
//}
//
//
//void TableModeler::segmentInteractionRegion(PointCloudPtr pCloudA,
//                                            PointCloudPtr pCloudB,
//                                            PointCloud& cloudObjsA,
//                                            PointCloud& cloudObjsB)
//{
//	segmentInteractionRegion(pCloudA, m_MinA, m_MaxA, m_OffsetA, m_ytonA, m_ytonAInv, cloudObjsA);
//	segmentInteractionRegion(pCloudB, m_MinB, m_MaxB, m_OffsetB, m_ytonB, m_ytonBInv, cloudObjsB);
//}
//
//
//void TableModeler::segmentInteractionRegion(PointCloudPtr pCloud,
//                                            Point min, Point max,
//                                            float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv,
//                                            PointCloud& cloudObjs)
//{
//	PointCloudPtr pCloudAux (new PointCloud());
//	PointCloudPtr pCloudAuxF (new PointCloud());
//
//	// Transformation
//	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
//    
////    pcl::visualization::PCLVisualizer viz ("hola");
////    viz.addCoordinateSystem();
////    int c = 0;
////    
////    viz.addCube(min.x, max.x, min.y, max.y + m_OffsetA, min.z, max.z, 1, 1, 1, "cube");
////    viz.addCube(min.x - m_InteractionBorder, max.x + m_InteractionBorder,
////                 max.y, max.y + 1.5,
////                 min.z - m_InteractionBorder, max.z + m_InteractionBorder, 1, 0, 0, "cube2");
//    
//    pcl::ConditionalRemoval<Point> condrem;
//    
//    // build the condition
//    pcl::ConditionAnd<Point>::Ptr inInteractionRegionRangeCondition (new pcl::ConditionAnd<Point> ());
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::GT, min.x - m_InteractionBorder)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::LT, max.x + m_InteractionBorder)));
//    
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::GT, max.y)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::LT, max.y + offset + m_InteractionBorder)));
//    
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::GT, min.z - m_InteractionBorder)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::LT, max.z + m_InteractionBorder)));
//    
//    condrem.setCondition(inInteractionRegionRangeCondition);
//    
//    condrem.setInputCloud (pCloudAux);
//    condrem.setKeepOrganized(true);
//    condrem.filter (*pCloudAuxF);
//    
//    
//    pCloudAuxF.swap(pCloudAux);
//    
//    pcl::ConditionOr<Point>::Ptr outTableTopRangeCondition (new pcl::ConditionOr<Point> ());
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::LT, min.x)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("x", pcl::ComparisonOps::GT, max.x)));
//    
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::LT, max.y)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("y", pcl::ComparisonOps::GT, max.y + offset)));
//    
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::LT, min.z)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<Point>::ConstPtr (new pcl::FieldComparison<Point> ("z", pcl::ComparisonOps::GT, max.z)));
//    
//    condrem.setCondition(outTableTopRangeCondition);
//    condrem.setInputCloud (pCloudAux);
//    condrem.setKeepOrganized(true);
//    condrem.filter (*pCloudAuxF);
//    
//	// De-transformation
//    
////    cout << "out tabletop filtered" << endl;
////    viz.removeAllPointClouds();
////    viz.addPointCloud (pCloudAuxF, "cloud left");
////    c = 0;
////    while(c++ < 10)
////    {
////        viz.spinOnce(1000);
////    }
//    
//	pcl::transformPointCloud(*pCloudAuxF, cloudObjs, ytonInv);
//}
//
//void TableModeler::read(string path, string filename, string extension)
//{
//    ifstream file;
//    string filepath = path + filename + "." + extension;
//    file.open(filepath, ios::in);
//    if (file.is_open())
//    {
//        cout << "Reading plane from A ... " << endl;
//        
//        file >> m_MinA.x >> m_MinA.y >> m_MinA.z;
//        file >> m_MaxA.x >> m_MaxA.y >> m_MaxA.z;
//        
//        m_pPlaneCoeffsA->values.resize(4);
//        file >> m_pPlaneCoeffsA->values[0] >> m_pPlaneCoeffsA->values[1] >> m_pPlaneCoeffsA->values[2] >> m_pPlaneCoeffsA->values[3];
//        computeTransformation(m_pPlaneCoeffsA, m_ytonA);
//        m_ytonAInv = m_ytonA.inverse();
//        
//        // Debug
//        cout << "minA : " << m_MinA << endl;
//        cout << "maxA : " << m_MaxA << endl;
//        cout << "coefficientsA : (" << m_pPlaneCoeffsA->values[0] << "," << m_pPlaneCoeffsA->values[1] << "," << m_pPlaneCoeffsA->values[2] << "," << m_pPlaneCoeffsA->values[3] << ")" << endl;
//        
//        
//        cout << "Reading plane from B ... " << endl;
//        
//        file >> m_MinB.x >> m_MinB.y >> m_MinB.z;
//        file >> m_MaxB.x >> m_MaxB.y >> m_MaxB.z;
//        
//        m_pPlaneCoeffsB->values.resize(4);
//        file >> m_pPlaneCoeffsB->values[0] >> m_pPlaneCoeffsB->values[1] >> m_pPlaneCoeffsB->values[2] >> m_pPlaneCoeffsB->values[3];
//        computeTransformation(m_pPlaneCoeffsB, m_ytonB);
//        m_ytonBInv = m_ytonB.inverse();
//        
//        // Debug
//        cout << "minB : " << m_MinB << endl;
//        cout << "maxB : " << m_MaxB << endl;
//        cout << "coefficientsB : (" << m_pPlaneCoeffsB->values[0] << "," << m_pPlaneCoeffsB->values[1] << "," << m_pPlaneCoeffsB->values[2] << "," << m_pPlaneCoeffsB->values[3] << ")" << endl;
//    }
//}
//
//void TableModeler::write(string path, string filename, string extension)
//{
//    ofstream file;
//    string filepath = path + filename + "." + extension;
//    file.open(filepath, ios::out);
//    file << m_MinA.x << " " << m_MinA.y << " " << m_MinA.z << endl;
//    file << m_MaxA.x << " " << m_MaxA.y << " " << m_MaxA.z << endl;
//    file << m_pPlaneCoeffsA->values[0] << " " << m_pPlaneCoeffsA->values[1] << " " << m_pPlaneCoeffsA->values[2] << " " << m_pPlaneCoeffsA->values[3] << endl;
//    
//    file << m_MinB.x << " " << m_MinB.y << " " << m_MinB.z << endl;
//    file << m_MaxB.x << " " << m_MaxB.y << " " << m_MaxB.z << endl;
//    file << m_pPlaneCoeffsB->values[0] << " " << m_pPlaneCoeffsB->values[1] << " " << m_pPlaneCoeffsB->values[2] << " " << m_pPlaneCoeffsB->values[3] << endl;
//}

