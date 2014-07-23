//
//  ReMedi.h
//  remedi2
//
//  Created by Albert Clapés on 13/07/14.
//
//

#ifndef remedi2_ReMedi_h
#define remedi2_ReMedi_h

#include "Sequence.h"
#include "ColorDepthFrame.h"

#include "InteractiveRegisterer.h"
#include "TableModeler.h"
// TODO adapt these:
//#include "BackgroundSubtractor.h"
//#include "Monitorizer.h"

using namespace std;

class ReMedi
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
    ReMedi();
    ReMedi(const ReMedi& rhs);
    
    ReMedi& operator=(const ReMedi& rhs);
    
    /** \brief Set the sequences of frames used to learn the background model, for its further subtraction
     *  \param pBgSeq Background sequence
     */
    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pBgSeq);
    
    /** \brief Set the list of sequences to be processed
     *  \param pSequences Input sequences
     */
    void setInputSequences(vector<Sequence<ColorDepthFrame>::Ptr> pSequences);
    
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
    void setRegistererParameters(int p, int fid, int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius);
    
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
    void setTableModelerParameters(int fid, float leafsz, float normrad, int sacIters, float sacDist, float yoffset, float border, int confidence);
    
    // Set the parameters of the background subtractor
    void setSubtractorParameters(int n, int modality, int k, float lrate, float bg, int opening);
    // Set the parameters of the monitorizer
    void setMonitorizerParameters(float leafsz, float clusterDist);
    
    // Initialization (common factor from normal run and validation procedures)
    void initialize();
    
    // Normal functioning of the system
    void run();
    
    // Validation of the modules
    void validateBS(vector< vector<double> > parameters);
    
    enum { COLOR = 0, DEPTH = 1, COLORDEPTH = 2, COLORDEPTHNORMALS = 3 };
    
private:
    Sequence<ColorDepthFrame>::Ptr m_pBgSeq;
    vector<Sequence<ColorDepthFrame>::Ptr> m_pSequences;
    
    InteractiveRegisterer::Ptr m_pRegisterer;
    bool m_bSetRegistererCorrespondences;
    
    TableModeler::Ptr m_pTableModeler;

    // TODO adapt these:
//    BackgroundSubtractor::Ptr m_pSubtractor;
//    Monitorizer:Ptr m_pMonitorizer;
};


#endif
