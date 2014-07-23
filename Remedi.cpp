//
//  ReMedi.cpp
//  remedi2
//
//  Created by Albert Clapés on 13/07/14.
//
//

#include "ReMedi.h"

ReMedi::ReMedi()
: m_pRegisterer(new InteractiveRegisterer),
  m_bSetRegistererCorrespondences(false),
  m_pTableModeler(new TableModeler)
{
    
}

ReMedi::ReMedi(const ReMedi& rhs)
{
    *this = rhs;
}

ReMedi& ReMedi::operator=(const ReMedi& rhs)
{
    if (this != &rhs)
    {
        m_pBgSeq = rhs.m_pBgSeq;
        m_pSequences = rhs.m_pSequences;
        
        m_pRegisterer = rhs.m_pRegisterer;
        m_bSetRegistererCorrespondences = rhs.m_bSetRegistererCorrespondences;
        m_pTableModeler = rhs.m_pTableModeler;
//        m_pSubtractor = rhs.m_pSubtractor;
//        m_pMonitorizer = rhs.m_pMonitorizer;
    }
    
    return *this;
}


/** \brief Set the sequences of frames used to learn the background model, for its further subtraction
 *  \param pBgSeq Background sequence
 */
void ReMedi::setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pBgSeq)
{
    m_pBgSeq = pBgSeq;
}

/** \brief Set the list of sequences to be processed
 *  \param pSequences Input sequences
 */void ReMedi::setInputSequences(vector<Sequence<ColorDepthFrame>::Ptr> pSequences)
{
    m_pSequences = pSequences;
}

/** \brief Set the parameters of the interactive registerer
 * \param  p : number of points used to compute the orthogonal transformation among views
 * \param  fid frame identifier (numerical value)
 * \param  wndHeight : window height (visor)
 * \param  wndWidth : window width (visor)
 * \param  vp : number of vertical ports (visor)
 * \param  hp : number of horizontal ports (visor)
 * \param  camDist : camera dist to (0,0,0) (visor)
 * \param  markerRadius : marker sphere radius (visor)
 */
void ReMedi::setRegistererParameters(int p, int fid, int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius)
{
    m_bSetRegistererCorrespondences = (p != -1); // 3 is the minimum num of correspondences to compute a transformation in a 3D space
    m_pRegisterer->setNumPoints(p);
    
    vector<ColorDepthFrame::Ptr> frames = m_pBgSeq->getFrames(fid);
    m_pRegisterer->setInputFrames(frames);
    
    m_pRegisterer->setVisualizerParameters(wndHeight, wndWidth, vp, hp, camDist, markerRadius);
}

/** \brief Set the parameters of the table modeler
 * \param  fid : frame identifier (numerical value)
 * \param  leafsz : leaf size in the voxel grid downsampling (speeds up the normal computation)
 * \param  normrad : neighborhood radius in normals estimation
 * \param  sacIters : num of iterations in RANSAC used for plane estimation
 * \param  sacDist : distance threshold to consider outliers in RANSAC
 * \param  yoffset : tabletop offset
 * \param  border : distance to the border table when considering blobs
 * \param  condifence : statistical removal of tabletop outlier points (along the y dimension)
 */
void ReMedi::setTableModelerParameters(int fid, float leafsz, float normrad, int sacIters, float sacDist, float yoffset, float border, int confidence)
{
    vector<ColorDepthFrame::Ptr> frames = m_pBgSeq->getFrames(fid);
    m_pTableModeler->setInputFrames(frames);
    
    m_pTableModeler->setLeafSize(leafsz);
    m_pTableModeler->setNormalRadius(normrad);
    m_pTableModeler->setSACIters(sacIters);
    m_pTableModeler->setSACDistThresh(sacDist);
    m_pTableModeler->setInteractionBorder(border);
    m_pTableModeler->setConfidenceLevel(confidence);
}

// Set the parameters of the background subtractor
//    n : num of samples used for modelling
//    modality: color, depth, color+depth, ...
//    k : max number of components per mixture in each pixel
//    lrate : learning rate for the adaptive background model
//    bg : background ratio
//    opening : 2*opening+1 is the size of the kernel convolved to open the bg mask
void ReMedi::setSubtractorParameters(int n, int modality, int k, float lrate, float bg, int opening)
{
    
}

// Set the parameters of the monitorizer
//    leafsz : leaf size in the voxel grid downsampling (speeds up further processes)
//    clusterDist : distance threshold in spatial clustering of 3-D blobs
void ReMedi::setMonitorizerParameters(float leafsz, float clusterDist)
{
    
}

void ReMedi::initialize()
{
    // Register views (and select new correspondences if it's the case)
    if (m_bSetRegistererCorrespondences)
    {
        m_pRegisterer->setCorrespondencesManually();
        m_pRegisterer->saveCorrespondences("registerer_correspondences", "pcd");
    }
    m_pRegisterer->loadCorrespondences("registerer_correspondences", "pcd");
    m_pRegisterer->computeTransformations();
    
    // dbg->
    vector<ColorDepthFrame::Ptr> pUnregFrames = m_pBgSeq->getFrames(2);
    
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pRegClouds;
    m_pRegisterer->registrate(pUnregFrames, pRegClouds);
    
    VisualizerPtr pViz (new Visualizer);
    pViz->addPointCloud(pRegClouds[0], "cloud0");
    pViz->addPointCloud(pRegClouds[1], "cloud1");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "cloud0");
    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud1");
    pViz->addCoordinateSystem(0.1, 0, 0, 0, "csg");
    pViz->spin();
    // <-dbg

    // Model the tables
    
    m_pTableModeler->model();
}

void ReMedi::run()
{
    initialize();
}

void ReMedi::validateBS(vector< vector<double> > parameters)
{
    initialize();
}