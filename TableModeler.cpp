#include "TableModeler.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/io/pcd_io.h>

#include "cvxtended.h"


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
        m_pFrames = rhs.m_pFrames;
        
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
    // Find...
    
    PointCloudPtr pTable (new PointCloud);
    PointCloudPtr pContourOrg (new PointCloud);
    pcl::ModelCoefficients::Ptr pTableCoefficients (new pcl::ModelCoefficients);
    
    extractTableContour(m_pFrames[0], 5, *pTable, *pTableCoefficients, *pContourOrg);
    
    PointT referencePoint = m_pFrames[0]->getReferencePoint();
    PointT contourPoint;
    findClosestContourPointToReference(pContourOrg, referencePoint, contourPoint);
    
    // Find the transform: table's normal to (0,0,0) and its direction rotated to XY
    
    getTransformation(pContourOrg, pTableCoefficients, referencePoint, contourPoint, m_Transformation);
    
    // Transform the plane
    
    PointCloudPtr pTableTransformed (new PointCloud);
    pcl::transformPointCloud(*pTable, *pTableTransformed, m_Transformation);
    
    // Get the table's 3D surrounding bounding box

    getMinMax3D(pTableTransformed, 1, m_ConfidenceLevel, m_Min, m_Max);
    
    // Visualize
    
    PointCloudPtr pCloud (new PointCloud);
    m_pFrames[0]->getPointCloud(*pCloud);
    
    PointCloudPtr pCloudTransformed (new PointCloud);
    pcl::transformPointCloud(*pCloud, *pCloudTransformed, m_Transformation);
    
    visualizePlaneEstimation(pCloud, pTable, pCloudTransformed, pTableTransformed, m_Min, m_Max);
}

void TableModeler::segmentTop(DepthFrame::Ptr pFrame, cv::Mat& top)
{
    PointCloudPtr pRegisteredCloud (new PointCloud);
    pFrame->getRegisteredPointCloud(*pRegisteredCloud);
    
    PointCloudPtr pTopRegisteredCloud (new PointCloud);
    segmentTop(pRegisteredCloud, *pTopRegisteredCloud);
    
    PointCloudPtr pTopCloud (new PointCloud);
    pFrame->getDeregisteredPointCloud(pTopRegisteredCloud, *pTopCloud);
    
    cv::Mat aux;
    PointCloudToMat(pTopCloud, pFrame->getResY(), pFrame->getResX(), aux);
    
    cvx::close(aux, 1, top);
}

void TableModeler::segmentTop(PointCloudPtr pCloud, PointCloud& top)
{
    PointCloudPtr pCloudTransformed (new PointCloud);
    pcl::transformPointCloud(*pCloud, *pCloudTransformed, m_Transformation);
    
    PointT min (m_Min.x, m_Min.y - 0.05, m_Min.z); // 5 mm
    PointT max (m_Max.x, m_Max.y + m_YOffset, m_Max.z);
    
    PointCloudPtr pTopTransformed (new PointCloud);
    PointCloudPtr pAux (new PointCloud); // top's complementary points
    filter(pCloudTransformed, min, max, *pTopTransformed, *pAux);
    
    pcl::transformPointCloud(*pTopTransformed, top, m_Transformation.inverse());
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
//    pcl::StatisticalOutlierRemoval<PointT> sor;
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

void TableModeler::estimate(PointCloudPtr pCloud, float leafSize, PointT reference, PointCloud& planeSegmented, PointCloud& nonPlaneSegmented, pcl::ModelCoefficients& coefficients)
{
	// downsampling

    PointCloudPtr pCloudFiltered (new PointCloud);
    
    if (leafSize == 0)
        *pCloudFiltered = *pCloud;
    else
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud (pCloud);
        sor.setLeafSize (m_LeafSize, m_LeafSize, m_LeafSize);
        sor.setSaveLeafLayout(true);
        sor.filter(*pCloudFiltered);
    }

	// normal estimation

	pcl::PointCloud<pcl::Normal>::Ptr pNormalsFiltered (new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(pCloudFiltered);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (m_NormalRadius); // neighbors in a sphere of radius X meter
    ne.compute (*pNormalsFiltered);

	// model estimation

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sac (false);
	sac.setOptimizeCoefficients (true); // optional
    sac.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType (pcl::SAC_RANSAC);
    sac.setMaxIterations (m_SACIters);
    sac.setDistanceThreshold (m_SACDistThresh);

    // create filtering object
    
    pcl::ExtractIndices<PointT> pointExtractor;
    pcl::ExtractIndices<pcl::Normal> normalExtractor;
    
    pointExtractor.setKeepOrganized(true);
    normalExtractor.setKeepOrganized(true);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    PointCloudPtr pPlane (new PointCloud);
    PointCloudPtr pNonPlane (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr pNonPlaneNormals (new pcl::PointCloud<pcl::Normal>);
    
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
        
        // dbg <--
//        Visualizer vis;
//        vis.addPointCloud(pPlane, "plane");
//        vis.addPointCloud(pNonPlane, "nonplane");
//        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "plane");
//        vis.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "nonplane");
//        vis.spin();
        // dbg -->
        
		if ( (found = isCloudIncludingReference(pPlane, leafSize, reference)) )
		{
            // Prepare to return the segmented plane
            PointCloudPtr pPlaneFiltered (new PointCloud);
            
            pcl::StatisticalOutlierRemoval<PointT> sor;
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
//			pcl::StatisticalOutlierRemoval<PointT> sor;
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
void TableModeler::getTransformation(PointCloudPtr pPlane, pcl::ModelCoefficients::Ptr pCoefficients, PointT referencePoint, PointT contourPoint, Eigen::Affine3f& T)
{
    Eigen::Vector3f n (pCoefficients->values[0], pCoefficients->values[1], pCoefficients->values[2]);
    //n = -n; // if it is upside down, but it is not now :D
    Eigen::Vector3f reference = referencePoint.getVector3fMap();
    Eigen::Vector3f u = contourPoint.getVector3fMap() - reference;
    
    //pcl::getTransformationFromTwoUnitVectorsAndOrigin(n.normalized(), v.normalized(), -origin, T);
    pcl::getTransformationFromTwoUnitVectorsAndOrigin(n, u, reference, T);
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
void TableModeler::getMinMax3D(PointCloudPtr pCloud, int d, int confidence, PointT& min, PointT& max)
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
bool TableModeler::isCloudIncludingOrigin(PointCloudPtr pPlane, float leafSize)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const PointT & p =  pPlane->points[i];
        float dist = sqrtf(powf(p.x,2)+powf(p.y,2)+powf(p.z,2));
        
		if ( dist > 0 && dist < 2 * leafSize)
			return true;
	}
    
	return false;
}

/** \brief Determines wheter or not a cloud includes the reference point
 *  \param pCloud A cloud
 *  \param reference A reference point
 *  \return Whether or not includes the reference point
 */
bool TableModeler::isCloudIncludingReference(PointCloudPtr pPlane, float leafSize, PointT reference)
{
	for (int i = 0; i < pPlane->points.size(); i++)
	{
		// Is there any point in the camera viewpoint direction (or close to)?
		// That is having x and y close to 0
		const PointT & p =  pPlane->points[i];
        float dist = sqrtf(powf(p.x - reference.x, 2) + powf(p.y - reference.y, 2) + powf(p.z - reference.z, 2));
        
		if ( dist > 0 && dist < 2 * leafSize)
			return true;
	}
    
	return false;
}

/** \brief Filter the points given a 3D bounding box
 *  \param pCloud A cloud to filter
 *  \param min The minimum of the box
 *  \param max The maximum of the box
 *  \param cloudFiltered Points of the cloud kept (insiders)
 *  \param cloudRemoved Points of the cloud removed (outsiders)
 */
void TableModeler::filter(PointCloudPtr pCloud, PointT min, PointT max, PointCloud& cloudFiltered, PointCloud& cloudRemoved)
{
    // build the condition for "within tabletop region"
    pcl::ConditionAnd<PointT>::Ptr condition (new pcl::ConditionAnd<PointT> ());
    
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, min.x)));
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, max.x)));
    
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, min.y)));
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, max.y)));
    
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, min.z)));
    condition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, max.z)));
    
    // build the filter
    pcl::ConditionalRemoval<PointT> removal (condition, true); // true = keep indices from filtered points
    removal.setInputCloud (pCloud);
    removal.filter(cloudFiltered);
    
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(pCloud);
    extractor.setIndices(removal.getRemovedIndices());
    extractor.filter(cloudRemoved);
}

/** \brief Extract the contour of the potential table plane
 *  \param pDepthFrame Frame in which the table plane is present
 *  \param max closingLevel In case the table plane cloud is containing small holes
 *  \param tableCloud The extracted table plane
 *  \param tableCoefficients The four table cofficients, i.e. Ax + By + Cz + D = 0
 *  \param tableContourCloud The organized cloud representing the table plane's contour line
 */
void TableModeler::extractTableContour(DepthFrame::Ptr pDepthFrame, int closingLevel, PointCloud& table, pcl::ModelCoefficients& tableCoefficients, PointCloud& contourCloud)
{
    PointCloudPtr pCloud (new PointCloud);
    pDepthFrame->getPointCloud(*pCloud);
    
    PointT ref = pDepthFrame->getReferencePoint();
    
    PointCloudPtr pNonTable (new PointCloud); // not used
    estimate(pCloud, 0.005, ref, table, *pNonTable, tableCoefficients);
    
    PointCloudPtr pTable (new PointCloud);
    *pTable = table;
    
    cv::Mat tableMat;
    PointCloudToMat(pTable, pDepthFrame->getResY(), pDepthFrame->getResX(), tableMat);
    
    cv::Mat tableOpenendMask, tableErodedMask;
    cvx::close(tableMat > 0, closingLevel, tableOpenendMask);
    cvx::erode(tableOpenendMask, 3, tableErodedMask); // hardcoded
    
    vector< vector<cv::Point> > contours;
    cv::findContours(tableErodedMask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    cv::Mat contourMask (tableErodedMask.size(), CV_8UC1, cv::Scalar(0));
    cv::drawContours(contourMask, contours, 0, cv::Scalar(255)); // 0-th should be the more external contour
    
    PointCloudPtr pContourCloud (new PointCloud);
    pDepthFrame->getPointCloud(contourMask, contourCloud);
}

/** \brief Find the closest point of the countour to the reference point
 *  \param pContourCloud The organized cloud representing a contour line
 *  \param referencePoint Reference point
 *  \param nearestContourPoint Point of contour cloud closest to the refrence
 */
void TableModeler::findClosestContourPointToReference(PointCloudPtr pContourCloud, PointT referencePoint, PointT& closestPoint)
{
    int minIdx;
    float minDist = std::numeric_limits<float>::infinity();
    for (int i = 0; i < pContourCloud->points.size(); i++)
    {
        if (pContourCloud->points[i].z > 0)
        {
            float dist = sqrt(pow(pContourCloud->points[i].x - referencePoint.x, 2) + pow(pContourCloud->points[i].y - referencePoint.y, 2) + pow(pContourCloud->points[i].z - referencePoint.z, 2));
            
            if (dist < minDist)
            {
                minDist = dist;
                minIdx = i;
            }
        }
    }
    
    closestPoint = pContourCloud->points[minIdx];
}

void TableModeler::visualizePlaneEstimation(PointCloudPtr pScene, PointCloudPtr pTable, PointCloudPtr pSceneTransformed, PointCloudPtr pTableTransformed, PointT min, PointT max)
{
    pcl::visualization::PCLVisualizer pViz;
    pViz.addCoordinateSystem();

    pViz.addPointCloud(pScene, "scene");
    pViz.addPointCloud(pTable, "plane");
    
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
//void TableModeler::segmentTableTop(PointCloudPtr pCloud, PointT min, PointT max, float offset, Eigen::Affine3f yton, Eigen::Affine3f ytonInv, PointCloud& cloudObjs)
//{
//	PointCloudPtr pCloudAux (new PointCloud());
//	PointCloudPtr pCloudAuxF (new PointCloud());
//    
//	// Transformation
//	pcl::transformPointCloud(*pCloud, *pCloudAux, yton);
//	
//    // build the condition
//    pcl::ConditionAnd<PointT>::Ptr inTableTopRegionCondition (new pcl::ConditionAnd<PointT> ());
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, min.x)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, max.x)));
//    
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, max.y)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, max.y + offset)));
//    
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, min.z)));
//    inTableTopRegionCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, max.z)));
//    
//    // build the filter
//    pcl::ConditionalRemoval<PointT> condrem (inTableTopRegionCondition);
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
//                                            PointT min, PointT max,
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
//    pcl::ConditionalRemoval<PointT> condrem;
//    
//    // build the condition
//    pcl::ConditionAnd<PointT>::Ptr inInteractionRegionRangeCondition (new pcl::ConditionAnd<PointT> ());
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, min.x - m_InteractionBorder)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, max.x + m_InteractionBorder)));
//    
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, max.y)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, max.y + offset + m_InteractionBorder)));
//    
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, min.z - m_InteractionBorder)));
//    inInteractionRegionRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, max.z + m_InteractionBorder)));
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
//    pcl::ConditionOr<PointT>::Ptr outTableTopRangeCondition (new pcl::ConditionOr<PointT> ());
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, min.x)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, max.x)));
//    
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, max.y)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, max.y + offset)));
//    
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, min.z)));
//    outTableTopRangeCondition->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, max.z)));
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

