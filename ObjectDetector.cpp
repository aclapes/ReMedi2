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
#include <pcl/filters/voxel_grid.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

ObjectDetector::ObjectDetector()
: m_MorphLevel(1), m_LeafSize(0.01), m_ClusterIdF(2), m_MinClusterSize(100), m_CorrespenceDist(0.1), m_bRegistration(false)
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
        m_InputFrames = rhs.m_InputFrames;
        
        m_MorphLevel = rhs.m_MorphLevel;
        m_LeafSize = rhs.m_LeafSize;
        m_ClusterIdF = rhs.m_ClusterIdF;
        m_MinClusterSize = rhs.m_MinClusterSize;
        m_CorrespenceDist = rhs.m_CorrespenceDist;
        
        m_bRegistration = rhs.m_bRegistration;
        
        m_Detections = rhs.m_Detections;
    }
    return *this;
}

void ObjectDetector::setInputFrames(vector<ColorDepthFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

void ObjectDetector::setMorhologyLevel(int level)
{
    m_MorphLevel = level;
}

void ObjectDetector::setDownsamplingSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void ObjectDetector::setClusteringIntradistanceFactor(float idf)
{
    m_ClusterIdF = idf;
}

void ObjectDetector::setMinClusterSize(int minSize)
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

void ObjectDetector::detect(vector<vector<ColorPointCloudPtr> >& detections)
{
    m_Detections.resize(m_InputFrames.size());
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
        ColorPointCloudPtr pColorCloud (new ColorPointCloud);
        m_InputFrames[v]->getColoredPointCloud(foreground, *pColorCloud);
        
        // Preprocess (at cloud level)
        ColorPointCloudPtr pColorCloudFiltered (new ColorPointCloud);
        downsample(pColorCloud, m_LeafSize, *pColorCloudFiltered); // cloud downsampling
        
        // Clustering
        vector<ColorPointCloudPtr> clusters;
        clusterize(pColorCloudFiltered, m_LeafSize, m_ClusterIdF, m_MinClusterSize, clusters);
        
        m_Detections[v] = clusters;
    }
    
    detections = m_Detections;
}

void ObjectDetector::getDetectionPositions(vector<vector<PointT> >& positions)
{
    _getDetectionPositions(m_Detections, m_bRegistration, positions);
}

void ObjectDetector::getDetectionCorrespondences(vector<vector<pair<int,ColorPointCloudPtr> > >& correspondences)
{
    findCorrespondences(m_Detections, m_CorrespenceDist, correspondences);
}

void ObjectDetector::_getDetectionPositions(vector<vector<ColorPointCloudPtr> > detections, bool bRegistrate, vector<vector<PointT> >& positions)
{
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
            
//            viewPositions[i].getVector4fMap() = centroid;
//            if (bRegistrate)
//                viewPositions[i] = m_InputFrames[v]->registratePoint(viewPositions[i]);
        }
        
        positions[v] = viewPositions;
    }
}

void ObjectDetector::clusterize(ColorPointCloudPtr pCloud, float leafSize, float clusterIdF, int minSize, vector<ColorPointCloudPtr>& clusters)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<ColorPointT>::Ptr tree (new pcl::search::KdTree<ColorPointT>);
    tree->setInputCloud (pCloud);
    
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<ColorPointT> ec;
    ec.setClusterTolerance (clusterIdF * leafSize);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (X_RESOLUTION * Y_RESOLUTION);
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
