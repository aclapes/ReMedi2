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

#include <pcl/common/norms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

Monitorizer::Monitorizer()
: m_MorphLevel(1), m_LeafSize(0.01), m_ClusterIdF(2)
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

void Monitorizer::setInterviewCorrepondenceDistance(float d)
{
    m_CorrespenceDist = d;
}

void Monitorizer::detect(vector<vector<PointCloudPtr> >& detections)
{
    detections.resize(m_InputFrames.size());
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
        
        detections.push_back(clusters);
    }
}

void Monitorizer::detect(vector<vector<PointT> >& positions)
{
    vector<vector<PointCloudPtr> > detections;
    detect(detections);
    
    getDetectionsPositions(detections, positions);
}

void Monitorizer::getDetectionsPositions(vector<vector<PointCloudPtr> > detections, vector<vector<PointT> >& positions)
{
    positions.resize(detections.size());
    for (int v = 0; v < detections.size(); v++)
    {
        vector<PointT> viewPositions (detections[v].size());
        
        for (int i = 0; i < detections[v].size(); i++)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*(detections[v][i]), centroid);
            
            viewPositions[i].getVector4fMap() = centroid;
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

void Monitorizer::recognize(vector<vector<vector<PointT> > >& recognitions)
{
    vector<vector<PointCloudPtr> > detections;
    detect(detections);
    
    vector<CloudjectPtr> cloudjects;
    cloudjectify(detections, cloudjects);
    
    for (int i = 0; i < cloudjects.size(); i++)
    {
        
    }
}

void Monitorizer::cloudjectify(vector<vector<PointCloudPtr> > detections, vector<CloudjectPtr>& cloudjects)
{
    vector<vector<pair<int,PointCloudPtr> > > correspondences;
    findCorrespondences(detections, m_CorrespenceDist, correspondences);
    
    cloudjects.clear();
    for (int i = 0; i < correspondences.size(); i++)
    {
        CloudjectPtr pCloudject ( new Cloudject(correspondences[i]) );
        cloudjects += pCloudject;
    }
}

void Monitorizer::findCorrespondences(vector<vector<PointCloudPtr> > detections, float tol, vector<vector<pair<int,PointCloudPtr> > >& correspondences)
{
    // Correspondences found using the positions of the clouds' centroids
    vector<vector<PointT> > positions;
    getDetectionsPositions(detections, positions);
    
    // Keep already made assignations, to cut search paths
    vector<vector<bool> > assignations (positions.size());
    for (int v = 0; v < positions.size(); v++)
        assignations[v].resize(positions[v].size(), false);
    
    // Cumbersome internal (from this function) structure:
    // Vector of chains
    // A chain is a list of points
    // These points have somehow attached the 'view' and the 'index in the view'.
    vector<vector<pair<pair<int,int>,PointT> > > _correspondences;
    for (int v = 0; v < positions.size(); v++)
    {
        for (int i = 0; i < positions[v].size(); i++)
        {
            if (!assignations[v][i])
            {
                vector<pair<pair<int,int>,PointT> > chain; // points chain
                chain += pair<pair<int,int>,PointT>(pair<int,int>(v,i), positions[v][i]);
                assignations[v][i] = true;
                
                findNextCorrespondence(positions, assignations, v+1, tol, chain); // recursion
                _correspondences += chain;
            }
        }
    }
    
    // Bc we want to have chains of clouds (not points) and the views to which they correspond, transform _correspondences -> correspondences
    correspondences.clear();
    for (int i = 0; i < _correspondences.size(); i++)
    {
        vector<pair<int,PointCloudPtr> > chain; // clouds chain
        for (int j = 0; _correspondences[i].size(); j++)
        {
            int v   = _correspondences[i][j].first.first;
            int idx = _correspondences[i][j].first.second;
            
            chain += pair<int,PointCloudPtr>(v, detections[v][idx]);
        }
        correspondences += chain;
    }
}

void Monitorizer::findNextCorrespondence(vector<vector<PointT> >& positions, vector<vector<bool> >& assignations, int v, float tol, vector<pair<pair<int,int>,PointT> >& chain)
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
