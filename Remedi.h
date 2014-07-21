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
// TODO adapt these:
//#include "TableModeler.h"
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
    
    // Set the sequences of frames used to learn the background model, for its further subtraction
    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pBgSeq);
    // Set the list of sequences to be processed
    void setInputSequences(vector<Sequence<ColorDepthFrame>::Ptr> pSequences);
    
    // Set the parameters of the interactive registerer
    void setRegistererParameters(int p, int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius, int fid);
    // Set the parameters of the table modeler
    void setTableModelerParameters(float leafsz, float normrad, int sacIters, float sacDist, float yoffset, float border, int confidence);
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
    
    // TODO adapt these:
//    TableModeler::Ptr m_pTableModeler;
//    BackgroundSubtractor::Ptr m_pSubtractor;
//    Monitorizer:Ptr m_pMonitorizer;
};


#endif
