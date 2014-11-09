//
//  ObjectDetector.cpp
//  remedi2
//
//  Created by Albert Clapés on 24/08/14.
//
//

#include "ObjectDetector.h"

#include "cvxtended.h"
#include "conversion.h"
#include "constants.h"

#include <pcl/common/norms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

float g_ColorDist = 20;
float g_NormalDotThresh = 0.9;

bool e(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.normal, point_b_normal = point_b.normal;
    float d = sqrtf(powf(point_a.r - point_b.r,2) + powf(point_a.g - point_b.g,2) + powf(point_a.b - point_b.b,2));
    
    if (d < g_ColorDist && fabs (point_a_normal.dot (point_b_normal)) > g_NormalDotThresh)
        return (true);
    return (false);
}

ObjectDetector::ObjectDetector()
: m_LeafSize(0.01), m_ClusterIdF(2), m_MinClusterSize(1/100.f), m_CorrespenceDist(0.1), m_bRegistration(false)
{
    
}

ObjectDetector::ObjectDetector(const ObjectDetector& rhs)
{
    *this = rhs;
}

ObjectDetector& ObjectDetector::operator=(const ObjectDetector& rhs)
{
    if (this != &rhs)
    {
//        m_InputFrames = rhs.m_InputFrames;
        
        m_LeafSize = rhs.m_LeafSize;
        m_ClusterIdF = rhs.m_ClusterIdF;
        m_MinClusterSize = rhs.m_MinClusterSize;
        m_CorrespenceDist = rhs.m_CorrespenceDist;
        
        m_bRegistration = rhs.m_bRegistration;
        
//        m_Detections = rhs.m_Detections;
    }
    return *this;
}

void ObjectDetector::setInputFrames(vector<ColorDepthFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

void ObjectDetector::setActorMasks(vector<cv::Mat> masks)
{
    m_ActorMasks = masks;
}

void ObjectDetector::setInteractionMasks(vector<cv::Mat> masks)
{
    m_InteractionMasks = masks;
}

void ObjectDetector::setDownsamplingSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void ObjectDetector::setClusteringIntradistanceFactor(float idf)
{
    m_ClusterIdF = idf;
}

void ObjectDetector::setMinClusterSize(float minSize)
{
    m_MinClusterSize = minSize;
}

void ObjectDetector::setInterviewCorrepondenceDistance(float d)
{
    m_CorrespenceDist = d;
}

void ObjectDetector::setRegistration(bool registration)
{
    m_bRegistration = registration;
}

void ObjectDetector::detect()
{
    vector<vector<ColorPointCloudPtr> > tmp;
    detect(tmp);
}

void visualizeDetections(std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > detections)
{
    // Create visualizer
    pcl::visualization::PCLVisualizer::Ptr pVis ( new pcl::visualization::PCLVisualizer );
    
    // Create viewports (horizontally)
    vector<int> viewports (detections.size());
    for (int v = 0; v < detections.size(); v++)
        pVis->createViewPort(v * (1.f/detections.size()), 0, (v+1) * (1.f/detections.size()), 1, viewports[v]);
    
    for (int v = 0; v < detections.size(); v++)
    {
        for (int i = 0; i < detections[v].size(); i++)
        {
            pVis->addPointCloud(detections[v][i], "detection_" + to_str(v) + "-" + to_str(i), viewports[v]);
            pVis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, g_Colors[i%14][0], g_Colors[i%14][1], g_Colors[i%14][2], "detection_" + to_str(v) + "-" + to_str(i), viewports[v]);
        }
    }
    
    pVis->spin();
}

void ObjectDetector::merge(std::vector<ColorPointCloudPtr> detections, std::vector<ColorPointCloudPtr>& mergeds)
{
    mergeds.clear();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pCentroidCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pCentroidCloud->width = detections.size();
    pCentroidCloud->height = 1;
    pCentroidCloud->resize(pCentroidCloud->width * pCentroidCloud->height);
    
    vector<Eigen::Vector4f> centroids (detections.size());
    for (int i = 0; i < detections.size(); i++)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*detections[i], centroids[i]);
        pCentroidCloud->points[i].getVector4fMap() = centroids[i];
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr pTree (new pcl::search::KdTree<pcl::PointXYZ>);
    pTree->setInputCloud (pCentroidCloud);
    
    vector<pcl::PointIndices> clusterIndices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ce;
    ce.setMinClusterSize(1);
    ce.setMaxClusterSize(detections.size());
    ce.setClusterTolerance(0.1);
    ce.setSearchMethod(pTree);
    ce.setInputCloud(pCentroidCloud);
    ce.extract(clusterIndices);
    
    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
    {
        ColorPointCloudPtr pCluster (new ColorPointCloud);

        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            *pCluster += *(detections[*pit]); //*

        mergeds.push_back(pCluster);
    }
}

void ObjectDetector::detect(vector<vector<ColorPointCloudPtr> >& detections)
{
    m_Detections.clear();
    
    m_Detections.resize(m_InputFrames.size());
    for (int v = 0; v < m_InputFrames.size(); v++)
    {
        // Get the frame in which detection is performed (and mask)
        cv::Mat color = m_InputFrames[v]->getColor();
        cv::Mat depth = m_InputFrames[v]->getDepth();
        cv::Mat actorsfg = m_ActorMasks[v] & m_InputFrames[v]->getMask();
        cv::Mat interactors = m_InteractionMasks[v] & ~m_ActorMasks[v];
        
        // Convert to cloud form
        ColorPointCloudPtr pForegroundActorCloud (new ColorPointCloud);
        ColorPointCloudPtr pInteractorCloud (new ColorPointCloud);

        m_InputFrames[v]->getColoredPointCloud(actorsfg, *pForegroundActorCloud);
        m_InputFrames[v]->getColoredPointCloud(interactors, *pInteractorCloud);
        
//        pcl::visualization::PCLVisualizer pVis (std::to_string(v));
//        pVis.addPointCloud(pForegroundActorCloud, "foreground actor");
//        pVis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "foreground actor");
//        pVis.addPointCloud(pInteractorCloud, "interactor");
//        pVis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "interactor");
//        
//        pVis.spin();
        
        // Preprocess (at cloud level)
        ColorPointCloudPtr pForegroundActorFiltered (new ColorPointCloud);
        downsample(pForegroundActorCloud, m_LeafSize, *pForegroundActorFiltered); // cloud downsampling
        
        // Clustering
#ifdef OD_CONDITIONAL_CLUSTERING
        vector<ColorPointCloudPtr> clusters, _clusters;
        cclusterize(pForegroundActorFiltered, m_LeafSize, m_ClusterIdF, m_MinClusterSize, _clusters);
        merge(_clusters, clusters);
#else
        vector<ColorPointCloudPtr> clusters;
        clusterize(pForegroundActorFiltered, m_LeafSize, m_ClusterIdF, m_MinClusterSize, clusters);
#endif
//        m_Detections[v] = clusters;

        std::vector<int> indices;
        subtractInteractionFromActors(clusters, pInteractorCloud, indices, 3 * m_LeafSize); // leaf size of the most downsampled either actor of interactor
        
        for (int i = 0; i < indices.size(); i++)
            m_Detections[v].push_back(clusters[indices[i]]);
    }
    
    detections = m_Detections;
    
//    visualizeDetections(detections);
}

void ObjectDetector::getDetectionPositions(vector<vector<PointT> >& positions)
{
    positions.clear();
    
    _getDetectionPositions(m_Detections, m_bRegistration, positions);
}

//void ObjectDetector::getDetectionCorrespondences(vector<vector<pair<int,PointT> > >& positions)
//{
//    vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences;
//    findCorrespondences(m_Detections, bMakeCorrespondences ? m_CorrespenceDist : 0, correspondences);
//}

void ObjectDetector::getDetectionCorrespondences(vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences, bool bMakeCorrespondences)
{
    correspondences.clear();
    
    findCorrespondences(m_Detections, bMakeCorrespondences ? m_CorrespenceDist : 0, correspondences);
}

void ObjectDetector::_getDetectionPositions(vector<vector<ColorPointCloudPtr> > detections, bool bRegistrate, vector<vector<PointT> >& positions)
{
    positions.clear();
    
    positions.resize(detections.size());
    for (int v = 0; v < detections.size(); v++)
    {
        vector<PointT> viewPositions (detections[v].size());
        
        for (int i = 0; i < detections[v].size(); i++)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*(detections[v][i]), centroid);
            
            if (!bRegistrate)
                viewPositions[i].getVector4fMap() = centroid;
            else
                viewPositions[i].getVector4fMap() = m_InputFrames[v]->registratePoint(centroid);
        }
        
        positions[v] = viewPositions;
    }
}

void ObjectDetector::clusterize(ColorPointCloudPtr pCloud, float leafSize, float clusterIdF, float minSize, vector<ColorPointCloudPtr>& clusters)
{
    clusters.clear();
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<ColorPointT>::Ptr tree (new pcl::search::KdTree<ColorPointT>);
    tree->setInputCloud (pCloud);
    
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<ColorPointT> ec;
    ec.setClusterTolerance (clusterIdF);
    ec.setMinClusterSize (pCloud->points.size() * minSize);
    ec.setMaxClusterSize (pCloud->points.size());
    ec.setSearchMethod (tree);
    ec.setInputCloud (pCloud);
    ec.extract (cluster_indices);
    
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        ColorPointCloudPtr pCluster (new ColorPointCloud);
        
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            pCluster->points.push_back (pCloud->points[*pit]); //*
        
        pCluster->width = pCluster->points.size ();
        pCluster->height = 1;
        pCluster->is_dense = true;
        
        clusters.push_back(pCluster);
    }
}

void ObjectDetector::cclusterize(ColorPointCloudPtr pCloud, float leafSize, float clusterIdF, float minSize, vector<ColorPointCloudPtr>& clusters)
{
    clusters.clear();
    
    pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (pCloud);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (clusterIdF);
    ne.compute (*pNormals);
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pCloudWithNormals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*pCloud, *pNormals, *pCloudWithNormals);
    
    pcl::IndicesClustersPtr indicesClusters (new pcl::IndicesClusters);
    
    // Set up a Conditional Euclidean Clustering class
    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (pCloudWithNormals);
    cec.setConditionFunction (&e);
    cec.setClusterTolerance (clusterIdF);
    cec.setMinClusterSize (pCloud->points.size () * minSize);
    cec.setMaxClusterSize (pCloud->points.size () );
    cec.segment (*indicesClusters);
//    cec.getRemovedClusters (small_clusters, large_clusters);

    for (pcl::IndicesClusters::const_iterator it = indicesClusters->begin (); it != indicesClusters->end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            pCluster->points.push_back (pCloud->points[*pit]); //*
        
        pCluster->width = pCluster->points.size ();
        pCluster->height = 1;
        pCluster->is_dense = true;
        
        clusters.push_back(pCluster);
    }
}

void ObjectDetector::downsample(ColorPointCloudPtr pCloud, float leafSize, ColorPointCloud& cloudFiltered)
{
    if (leafSize == 0.f)
    {
        cloudFiltered = *pCloud; // copy
    }
    else
    {
        pcl::VoxelGrid<ColorPointT> vg;
        vg.setInputCloud(pCloud);
        vg.setLeafSize(leafSize, leafSize, leafSize);
        vg.filter(cloudFiltered);
    }
}

void ObjectDetector::findCorrespondences(vector<vector<ColorPointCloudPtr> > detections, float tol, vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences)
{
    correspondences.clear();
    
    // Correspondences found using the positions of the clouds' centroids
    vector<vector<PointT> > positions;
    _getDetectionPositions(detections, true, positions); // registrate is "true"
    
    vector<vector<pair<pair<int,int>,PointT> > > _correspondences;
    _findCorrespondences(positions, tol, _correspondences);
    
    // Bc we want to have chains of clouds (not points) and the views to which they correspond, transform _correspondences -> correspondences
    correspondences.clear();
    for (int i = 0; i < _correspondences.size(); i++)
    {
        vector<pair<int,ColorPointCloudPtr> > chain; // clouds chain
        for (int j = 0; j < _correspondences[i].size(); j++)
        {
            int v   = _correspondences[i][j].first.first;
            int idx = _correspondences[i][j].first.second;
            
            chain += pair<int,ColorPointCloudPtr>(v, detections[v][idx]);
        }
        correspondences += chain;
    }
}

void ObjectDetector::_findCorrespondences(vector<vector<PointT> > positions, float tol, vector<vector<pair<pair<int,int>,PointT> > >& correspondences)
{
    correspondences.clear();
    
    // Keep already made assignations, to cut search paths
    vector<vector<bool> > assignations (positions.size());
    for (int v = 0; v < positions.size(); v++)
        assignations[v].resize(positions[v].size(), false);
    
    // Cumbersome internal (from this function) structure:
    // Vector of chains
    // A chain is a list of points
    // These points have somehow attached the 'view' and the 'index in the view'.
    for (int v = 0; v < positions.size(); v++)
    {
        for (int i = 0; i < positions[v].size(); i++)
        {
            if (!assignations[v][i])
            {
                vector<pair<pair<int,int>,PointT> > chain; // points chain
                chain += pair<pair<int,int>,PointT>(pair<int,int>(v,i), positions[v][i]);
                assignations[v][i] = true;
                
                if (tol > 0)
                    findNextCorrespondence(positions, assignations, v+1, tol, chain); // recursion
                
                correspondences += chain;
            }
        }
    }
}


void ObjectDetector::findNextCorrespondence(vector<vector<PointT> >& positions, vector<vector<bool> >& assignations, int v, float tol, vector<pair<pair<int,int>,PointT> >& chain)
{
    if (v == positions.size())
    {
        return;
    }
    else
    {
        int minIdx = -1;
        float minDist = std::numeric_limits<float>::max();
        
        for (int i = 0; i < chain.size(); i++)
        {
            for (int j = 0; j < positions[v].size(); j++)
            {
                if ( !assignations[v][j] )
                {
                    PointT p = chain[i].second;
                    PointT q = positions[v][j];
                    float d = sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
                    
                    if (d < minDist && d <= tol)
                    {
                        minIdx = j;
                        minDist = d;
                    }
                }
            }
        }
        
        if (minIdx >= 0)
        {
            chain += pair<pair<int,int>,PointT>(pair<int,int>(v,minIdx), positions[v][minIdx]);
            assignations[v][minIdx] = true;
        }
        
        findNextCorrespondence(positions, assignations, v+1, tol, chain);
    }
}

void ObjectDetector::subtractInteractionFromActors(std::vector<ColorPointCloudPtr> actorClusters, ColorPointCloudPtr pInteractionCloud, std::vector<int>& indices, float leafSize)
{
    indices.clear();
    
    ColorPointCloudPtr pInteractorFiltered (new ColorPointCloud);
    downsample(pInteractionCloud, leafSize, *pInteractorFiltered);
    
    std::vector<ColorPointCloudPtr> actorsFiltered (actorClusters.size());
    for (int i = 0; i < actorClusters.size(); i++)
    {
        ColorPointCloudPtr pActorFiltered (new ColorPointCloud);
        downsample(actorClusters[i], leafSize, *pActorFiltered);
        actorsFiltered[i] = pActorFiltered;
    }
    
    // Check for each cluster that lies on the table
    for (int i = 0; i < actorClusters.size(); i++)
    {
        if (!isInteractive(actorsFiltered[i], pInteractorFiltered, leafSize))
            indices.push_back(i);
    }
}

bool ObjectDetector::isInteractive(ColorPointCloudPtr tabletopRegionCluster, ColorPointCloudPtr interactionCloud, float leafSize)
{
    bool interactive = false;
    
    // if some of its points
    for (int k_i = 0; k_i < tabletopRegionCluster->points.size() && !interactive; k_i++)
    {
        ColorPointT p = tabletopRegionCluster->points[k_i];
        
        // some point of the interaction region.
        for (int k_j = 0; k_j < interactionCloud->points.size() && !interactive; k_j++)
        {
            ColorPointT q = interactionCloud->points[k_j];
            
            // If they are practically contiguous, they must had been part of the same cloud
            float pqDist = sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
            interactive = (pqDist <= 2.f * leafSize); // if tabletop cluster is stick to any cluster
            // in the interaction region, we say it is interactive
            // and become an interactor
        }
    }
    
    return interactive;
}
