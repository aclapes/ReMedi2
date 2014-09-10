//
//  ReMedi.cpp
//  remedi2
//
//  Created by Albert Clapés on 13/07/14.
//
//

#include "ReMedi.h"
#include "cvxtended.h"
#include "statistics.h"

#include <boost/timer.hpp>
#include <boost/filesystem.hpp>

ReMedi::ReMedi()
: m_fID(0),
  m_pRegisterer(new InteractiveRegisterer),
  m_bSetRegistererCorrespondences(false),
  m_pTableModeler(new TableModeler),
  m_pMonitorizer(new Monitorizer)
{
    m_pBackgroundSubtractor = BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr(new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
//    m_pBackgroundSubtractor = BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>::Ptr(new BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>);
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
        m_fID = rhs.m_fID;
        
        m_pRegisterer = rhs.m_pRegisterer;
        m_bSetRegistererCorrespondences = rhs.m_bSetRegistererCorrespondences;
        
        m_pTableModeler = rhs.m_pTableModeler;
        
        m_pBackgroundSubtractor = rhs.m_pBackgroundSubtractor;
        
        m_pMonitorizer = rhs.m_pMonitorizer;
    }
    
    return *this;
}

InteractiveRegisterer::Ptr ReMedi::getRegisterer()
{
    return m_pRegisterer;
}

TableModeler::Ptr ReMedi::getTableModeler()
{
    return m_pTableModeler;
}

BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr ReMedi::getBackgroundSubtractor()
{
    return m_pBackgroundSubtractor;
}

Monitorizer::Ptr ReMedi::getMonitorizer()
{
    return m_pMonitorizer;
}

/** \brief Set the sequences of frames used to learn the background model, for its further subtraction
 *  \param pBgSeq Background sequence
 */
void ReMedi::setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pBgSeq)
{
    m_pBgSeq = pBgSeq;
}

Sequence<ColorDepthFrame>::Ptr ReMedi::getBackgroundSequence()
{
    return m_pBgSeq;
}

/** \brief Set the list of sequences to be processed
 *  \param pSequences Input sequences
 */
void ReMedi::setInputSequences(vector<Sequence<ColorDepthFrame>::Ptr> pSequences)
{
    m_pSequences = pSequences;
}

/** \brief Set resolutions
 *  \param xres
 *  \param yres
 */
void ReMedi::setFramesResolution(int xres, int yres)
{
    m_XRes = xres;
    m_YRes = yres;
}

/** \brief Set an static frame for multiple purposes
 *  \param fid Frame numerical ID
 */
void ReMedi::setDefaultFrame(int fid)
{
    m_fID = fid;
}

/** \brief Set the parameters of the interactive registerer
 * \param  p : number of points used to compute the orthogonal transformation among views
 * \param  wndHeight : window height (visor)
 * \param  wndWidth : window width (visor)
 * \param  vp : number of vertical ports (visor)
 * \param  hp : number of horizontal ports (visor)
 * \param  camDist : camera dist to (0,0,0) (visor)
 * \param  markerRadius : marker sphere radius (visor)
 */
void ReMedi::setRegistererParameters(int p, int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius)
{
    m_bSetRegistererCorrespondences = (p != -1); // 3 is the minimum num of correspondences to compute a transformation in a 3D space
    m_pRegisterer->setNumPoints(p);
    
    m_pRegisterer->setVisualizerParameters(wndHeight, wndWidth, vp, hp, camDist, markerRadius);
}

/** \brief Set the parameters of the table modeler
 * \param  leafsz : leaf size in the voxel grid downsampling (speeds up the normal computation)
 * \param  normrad : neighborhood radius in normals estimation
 * \param  sacIters : num of iterations in RANSAC used for plane estimation
 * \param  sacDist : distance threshold to consider outliers in RANSAC
 * \param  yoffset : tabletop offset
 * \param  border : distance to the border table when considering blobs
 * \param  condifence : statistical removal of tabletop outlier points (along the y dimension)
 */
void ReMedi::setTableModelerParameters(float leafsz, float normrad, int sacIters, float sacDist, float yoffset, float border, int confidence)
{
    m_pTableModeler->setLeafSize(leafsz);
    m_pTableModeler->setNormalRadius(normrad);
    m_pTableModeler->setSACIters(sacIters);
    m_pTableModeler->setSACDistThresh(sacDist);
    m_pTableModeler->setYOffset(yoffset);
    m_pTableModeler->setInteractionBorder(border);
    m_pTableModeler->setConfidenceLevel(confidence);
}

/** \brief Set the parameters of the background subtractor
 *  \param n The number of samples used for modelling
 *  \param m The modality used to model the background (color, color with shadows, or depth)
 *  \param k The maximum number of components per mixture in each pixel
 *  \param lrate The learning rate for the adaptive background model
 *  \param bgratio The background ratio
 *  \param vargen The variance threshold to consider generation of new mixture components
 */
void ReMedi::setSubtractorParameters(int n, int m, int k, float lrate, float bgratio, float vargen)
{
    m_pBackgroundSubtractor->setFramesResolution(m_XRes, m_YRes);
    
    // If models have not to be computed, we save the background seq loading time
    m_pBackgroundSubtractor->setBackgroundSequence(m_pBgSeq, n);
    
    m_pBackgroundSubtractor->setModality(m);
    m_pBackgroundSubtractor->setNumOfMixtureComponents(k);
    m_pBackgroundSubtractor->setLearningRate(lrate);
    m_pBackgroundSubtractor->setBackgroundRatio(bgratio);
    m_pBackgroundSubtractor->setVarThresholdGen(vargen);
}

///** \brief Set the parameters of the background subtractor
// *  \param n The number of samples used for modelling
// *  \param m The modality used to model the background (color, color with shadows, or depth)
// *  \param f The maximum number of components per mixture in each pixel
// *  \param lrate The learning rate for the adaptive background model
// *  \param q The background ratio
// *  \param t The decision threshold based on the variance criterion
// */
//void ReMedi::setSubtractorParameters(int n, int m, int f, float lrate, float q, float t)
//{
//    m_pBackgroundSubtractor->setFramesResolution(m_XRes, m_YRes);
//    
//    // If models have not to be computed, we save the background seq loading time
//    m_pBackgroundSubtractor->setBackgroundSequence(m_pBgSeq, n);
//    
//    m_pBackgroundSubtractor->setModality(m);
//    m_pBackgroundSubtractor->setNumOfMaxFeatures(f);
//    m_pBackgroundSubtractor->setLearningRate(lrate);
//    m_pBackgroundSubtractor->setQuantizationLevels(q);
//    m_pBackgroundSubtractor->setDecisionThreshold(t);
//}

/** \brief Set the parameters of the monitorizer
 * \param morphSize : kernel size of the mathematical morphology operation
 * \param leafSize : leaf size in the voxel grid downsampling (speeds up further processes)
 * \param clusterDist : distance threshold in spatial clustering of 3-D blobs
 * \param minClusterSize : minimum number of points a cluster hast to have to be considered
 */
void ReMedi::setMonitorizerParameters(int morphSize, float leafSize, float clusterDist, int minClusterSize)
{
    m_pMonitorizer->setMorhologyLevel(morphSize);
    m_pMonitorizer->setDownsamplingSize(leafSize);
    m_pMonitorizer->setClusteringIntradistanceFactor(clusterDist);
    m_pMonitorizer->setMinClusterSize(minClusterSize);
}

void ReMedi::initialize()
{
    // Register views (and select new correspondences if it's the case)

    m_pRegisterer->setInputFrames( m_pBgSeq->getFrames(m_fID) );
    
    if (!m_bSetRegistererCorrespondences)
        m_pRegisterer->loadCorrespondences("registerer_correspondences", "pcd");
    else
    {
        m_pRegisterer->setCorrespondencesManually();
        m_pRegisterer->saveCorrespondences("registerer_correspondences", "pcd");
    }
    
    m_pRegisterer->computeTransformations();
    
    vector<ColorDepthFrame::Ptr> frames;
    m_pRegisterer->registrate(frames);

    // Model the tables
    
    m_pTableModeler->setInputFrames(frames);
    m_pTableModeler->model();
}

void ReMedi::run()
{
    initialize();
}

void ReMedi::loadSequences(string parent, vector<string>& names)
{
    const char* path = parent.c_str();
	if( boost::filesystem::exists( path ) )
	{
		boost::filesystem::directory_iterator end;
		boost::filesystem::directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( boost::filesystem::is_directory( *iter ) )
            {
                string nameStr = iter->path().stem().string();
                names.push_back(nameStr);
            }
        }
    }
}

void ReMedi::loadDirPaths(string parent, vector<string> seqNames, string subdir, vector<string> viewsDirs, vector< vector<string> >& paths)
{
 	paths.resize(seqNames.size());
    for (int s = 0; s < seqNames.size(); s++)
    {
        string pathTmp = parent + "/" + seqNames[s]  + "/" + subdir;
        
        vector<string> seqPaths (viewsDirs.size());
        for (int v = 0; v < viewsDirs.size(); v++)
            seqPaths[v] = pathTmp + viewsDirs[v];
        
        paths[s] = seqPaths;
    }
}

void ReMedi::loadDelaysFile(string parent, string filename, vector<vector<int> >& delays)
{
    ifstream inFile;
    string path = parent + filename;
    inFile.open(path, ios::in);
    
    assert (inFile.is_open());
    
    string line;
    while (getline(inFile, line))
    {
        vector<string> line_words;
        boost::split(line_words, line, boost::is_any_of(" "));
        
        assert(line_words.size() == 2);
        
        vector<string> delays_subwords;
        boost::split(delays_subwords, line_words[1], boost::is_any_of(","));
        
        vector<int> delaysInLine;
        for (int i = 0; i < delays_subwords.size(); i++)
            delaysInLine.push_back( stoi(delays_subwords[i]) );
        
        delays.push_back(delaysInLine);
    }
    
    inFile.close();
}
