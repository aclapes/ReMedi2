//
//  Monitorizer.cpp
//  remedi2
//
//  Created by Albert Clapés on 24/08/14.
//
//

#include "Monitorizer.h"
#include "cvxtended.h"
#include "conversion.h"
#include "constants.h"

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

Monitorizer::Monitorizer()
: m_MorphLevel(0), m_LeafSize(0.02), m_ClusterIdF(0.04)
{
    
}

void Monitorizer::setInputFrames(vector<ColorDepthFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

void Monitorizer::setMorhologyLevel(int level)
{
    m_MorphLevel = level;
}

void Monitorizer::setDownsamplingSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void Monitorizer::setClusteringIntradistanceFactor(float idf)
{
    m_ClusterIdF = idf;
}

void Monitorizer::setMinClusterSize(int minSize)
{
    m_MinClusterSize = minSize;
}

void Monitorizer::detect(vector<vector<PointT> >& positions)
{
    positions.resize(m_InputFrames.size());
    for (int v = 0; v < m_InputFrames.size(); v++)
    {
        // Get the frame in which detection is performed (and mask)
        cv::Mat color = m_InputFrames[v]->getColor();
        cv::Mat depth = m_InputFrames[v]->getDepth();
        cv::Mat foreground = m_InputFrames[v]->getMask();
        
        // Preprocess (at blob level), // mathematical morphology
        if (m_MorphLevel < 0)
            cvx::close(foreground, abs(m_MorphLevel), foreground);
        else
            cvx::open(foreground, m_MorphLevel, foreground);
        
        // Convert to cloud form
        PointCloudPtr pCloud (new PointCloud);
        MatToPointCloud(depth, foreground, *pCloud);
        
        // Preprocess (at cloud level)
        PointCloudPtr pCloudFiltered (new PointCloud);
        downsample(pCloud, m_LeafSize, *pCloudFiltered); // cloud downsampling
        
        // Clustering
        vector<PointCloudPtr> clusters;
        clusterize(pCloudFiltered, m_LeafSize, m_ClusterIdF, m_MinClusterSize, clusters);
        
        vector<PointT> viewPositions (clusters.size());
        for (int p = 0; p < clusters.size(); p++)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*(clusters[p]), centroid);
            
            viewPositions[p].getVector4fMap() = centroid;
        }
        positions[v] = viewPositions;
    }
}

void Monitorizer::clusterize(PointCloudPtr pCloud, float leafSize, float clusterIdF, int minSize, vector<PointCloudPtr>& clusters)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (pCloud);
    
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterIdF * leafSize);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (X_RESOLUTION * Y_RESOLUTION);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pCloud);
    ec.extract (cluster_indices);
    
    for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudPtr pCluster (new PointCloud);
        
        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            pCluster->points.push_back (pCloud->points[*pit]); //*
        
        pCluster->width = pCluster->points.size ();
        pCluster->height = 1;
        pCluster->is_dense = true;
        
        clusters.push_back(pCluster);
    }
}

void Monitorizer::downsample(PointCloudPtr pCloud, float leafSize, PointCloud& cloudFiltered)
{
    if (leafSize == 0.f)
    {
        cloudFiltered = *pCloud; // copy
    }
    else
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(pCloud);
        vg.setLeafSize(leafSize, leafSize, leafSize);
        vg.filter(cloudFiltered);
    }
}