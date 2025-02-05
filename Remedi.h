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
#include "BackgroundSubtractor.h"
#include "DetectionOutput.h"
#include "ObjectDetector.h"
#include "ObjectModel.hpp"
#include "ObjectRecognizer.h"

#include <boost/variant.hpp>

// TODO adapt these:
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
    typedef pcl::FPFHSignature33 FPFHSignature;
    typedef pcl::PFHRGBSignature250 PFHRGBSignature;
    typedef pcl::PointCloud<FPFHSignature> FPFHDescription;
    typedef pcl::PointCloud<FPFHSignature>::Ptr FPFHDescriptionPtr;
    typedef pcl::PointCloud<PFHRGBSignature> PFHRGBDescription;
    typedef pcl::PointCloud<PFHRGBSignature>::Ptr PFHRGBDescriptionPtr;
//    typedef LFCloudjectModel<ColorPointT,FPFHSignature> CloudjectModel;
//    typedef LFCloudjectModel<ColorPointT,FPFHSignature>::Ptr CloudjectModelPtr;
    
public:
    ReMedi();
    ReMedi(const ReMedi& rhs);
    
    ReMedi& operator=(const ReMedi& rhs);
    
    InteractiveRegisterer::Ptr getRegisterer();
    TableModeler::Ptr getTableModeler();
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr getBackgroundSubtractor();
    ObjectDetector::Ptr getObjectDetector();
    void* getObjectRecognizer();
    int getDescriptionType();
    
    /** \brief Set the sequences of frames used to learn the background model, for its further subtraction
     *  \param pBgSeq Background sequence
     */
    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pBgSeq);
    Sequence<ColorDepthFrame>::Ptr getBackgroundSequence();
    

    /** \brief Set the list of sequences to be processed
     *  \param pSequences Input sequences
     */
    void setInputSequences(vector<Sequence<ColorDepthFrame>::Ptr> pSequences);
    
    /** \brief Set resolutions
     *  \param xres
     *  \param yres
     */
    void setFramesResolution(int xres, int yres);
    
    /** \brief Set an static frame for multiple purposes
     *  \param fid Frame numerical ID
     */
    void setDefaultFrame(int fid);
    
    /** \brief Set the parameters of the interactive registerer
     * \param  p : number of points used to compute the orthogonal transformation among views
     * \param  wndHeight : window height (visor)
     * \param  wndWidth : window width (visor)
     * \param  vp : number of vertical ports (visor)
     * \param  hp : number of horizontal ports (visor)
     * \param  camDist : camera dist to (0,0,0) (visor)
     * \param  markerRadius : marker sphere radius (visor)
     */
    void setRegistererParameters(int p, int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius);
    
    /** \brief Set the parameters of the table modeler
     * \param  leafsz : leaf size in the voxel grid downsampling (speeds up the normal computation)
     * \param  normrad : neighborhood radius in normals estimation
     * \param  sacIters : num of iterations in RANSAC used for plane estimation
     * \param  sacDist : distance threshold to consider outliers in RANSAC
     * \param  yoffset : tabletop offset
     * \param  border : distance to the border table when considering blobs
     * \param  condifence : statistical removal of tabletop outlier points (along the y dimension)
     */
    void setTableModelerParameters(float leafsz, float normrad, int sacIters, float sacDist, float yoffset, float border, int confidence);
    
#ifdef BS_USE_MOG2
    /** \brief Set the parameters of the background subtractor
     *  \param n The number of samples used for modelling
     *  \param m The modality used to model the background (color, color with shadows, or depth)
     *  \param k The maximum number of components per mixture in each pixel
     *  \param lrate The learning rate for the adaptive background model
     *  \param bgratio The background ratio
     *  \param vargen The variance threshold to consider generation of new mixture components
     *  \param level The size (2*level+1) of the kernel convolved to perform mathematical morphology (opening) to the background mask
     */
    void setSubtractorParameters(int n, int m, int k, float lrate, float bgratio, float vargen, float opening);
#else
    /** \brief Set the parameters of the background subtractor
     *  \param n The number of samples used for modelling
     *  \param m The modality used to model the background (color, color with shadows, or depth)
     *  \param f The maximum number of components per mixture in each pixel
     *  \param lrate The learning rate for the adaptive background model
     *  \param q The background ratio
     *  \param t The decision threshold based on the variance criterion
     */
    void setSubtractorParameters(int n, int m, int f, float lrate, float q, float t, float opening);
#endif
    
    /** \brief Set the parameters of the background subtractor
     *  \param level The size (2*level+1) of the kernel convolved to perform mathematical morphology (opening) to the background mask
     */
    void setObjectDetectorParameters(float leafSize, float clusterDist, int minClusterSize);

    void setObjectRecognizerParameters(vector<ObjectModel<ColorPointT>::Ptr> objectModels, vector<float> objectRejections, int descriptionType, int recognitionStrategy);
    void setObjectRecognizerPfhParameters(float leafSize, float leafSizeModel, float normalRadius, float normalRadiusModel, float pfhRadius, float pfhRadiusModel, float ptScrRjtThresh);

    void initialize();
    
    // Normal functioning of the system
    void run();
    
//    // Validation of the modules
//    
//    void validateBS(vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences);
//    void showBsParametersPerformance(vector<Sequence<Frame>::Ptr> fgMasksSequences, string filePath);
////    void getBsParametersPerformance(string filePath, cv::Mat& combinations, cv::Mat& overlaps);
//    
//    void readBsValidationFile(string filePath, cv::Mat& combinations, cv::Mat& overlaps);
//    void getBsParametersPerformance(cv::Mat overlaps, cv::Mat& means, cv::Mat& stddevs);
//    void rankParametersCombinations(cv::Mat means, cv::Mat& R);
//    
//    void validateMonitorizerClustering(vector<Sequence<Frame>::Ptr> fgMasksSequences,
//                                       cv::Mat bsCombinations,
//                                       vector<vector<double> > monitorizerParameters,
//                                       vector<DetectionOutput> detectionGroundtruths);
//    void readMonitorizerValidationFile(string filePath, cv::Mat& combinations, vector<cv::Mat>& errors);
//    void getMonitorizerFScorePerformances(cv::Mat combinations, vector<cv::Mat> errors, cv::Mat& meanScores, cv::Mat& sdScores);
//    void showMonitorizerValidationSummary(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat meanScores, cv::Mat sdScores, bool bMinimize = false);
    
    static void loadSequences(std::string parent, std::vector<std::string>& names, std::vector<int>& sids);
    static void loadDirPaths(string parent, vector<string> seqNames, string subdir, vector<string> viewsDirs, vector< vector<string> >& paths);
    static void loadDelaysFile(string parent, string filename, vector<vector<int> >& delays);
//    static void loadObjectsModels(string parent, vector<int>& objectsIDs, vector<string>& objectsNames, vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >& objectsViews);
    static void loadObjectModels(const char* path, const char* pcdDir, vector<string> modelsNames,
                                 vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >& views);
    
    typedef boost::shared_ptr<ReMedi> Ptr;
    
private:
    Sequence<ColorDepthFrame>::Ptr m_pBgSeq;
    vector<Sequence<ColorDepthFrame>::Ptr> m_pSequences;
    int m_fID;
    
    int m_XRes, m_YRes;
    
    InteractiveRegisterer::Ptr m_pRegisterer;
    bool m_bSetRegistererCorrespondences;
    
    TableModeler::Ptr m_pTableModeler;
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr m_pBackgroundSubtractor;
//    BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>::Ptr m_pBackgroundSubtractor;
    
    ObjectDetector::Ptr m_pObjectDetector;
    
    int                 m_DescriptionType;
    void*               m_pObjectRecognizer;
};

#endif
