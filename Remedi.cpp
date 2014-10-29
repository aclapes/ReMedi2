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
  m_pObjectDetector(new ObjectDetector), 
  m_pObjectRecognizer(NULL)
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
        
        m_pObjectDetector = rhs.m_pObjectDetector;
        
        m_DescriptionType = rhs.m_DescriptionType;
        m_pObjectRecognizer = rhs.m_pObjectRecognizer;
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

ObjectDetector::Ptr ReMedi::getObjectDetector()
{
    return m_pObjectDetector;
}

int ReMedi::getDescriptionType()
{
    return m_DescriptionType;
}

void* ReMedi::getObjectRecognizer()
{
    return m_pObjectRecognizer;
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
void ReMedi::setSubtractorParameters(int n, int m, int k, float lrate, float bgratio, float vargen, float opening)
{
    m_pBackgroundSubtractor->setFramesResolution(m_XRes, m_YRes);
    
    // If models have not to be computed, we save the background seq loading time
    m_pBackgroundSubtractor->setBackgroundSequence(m_pBgSeq, n);
    
    m_pBackgroundSubtractor->setModality(m);
    m_pBackgroundSubtractor->setNumOfMixtureComponents(k);
    m_pBackgroundSubtractor->setLearningRate(lrate);
    m_pBackgroundSubtractor->setBackgroundRatio(bgratio);
    m_pBackgroundSubtractor->setVarThresholdGen(vargen);
    m_pBackgroundSubtractor->setOpeningSize(opening);
}

///** \brief Set the parameters of the background subtractor
// *  \param n The number of samples used for modelling
// *  \param m The modality used to model the background (color, color with shadows, or depth)
// *  \param f The maximum number of components per mixture in each pixel
// *  \param lrate The learning rate for the adaptive background model
// *  \param q The background ratio
// *  \param t The decision threshold based on the variance criterion
// */
//void ReMedi::setSubtractorParameters(int n, int m, int f, float lrate, float q, float t, float o)
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
//    m_pBackgroundSubtractor->setOpeningSize(o);
//}

/** \brief Set the parameters of the object detector
 * \param morphSize : kernel size of the mathematical morphology operation
 * \param leafSize : leaf size in the voxel grid downsampling (speeds up further processes)
 * \param clusterDist : distance threshold in spatial clustering of 3-D blobs
 * \param minClusterSize : minimum number of points a cluster hast to have to be considered
 */
void ReMedi::setObjectDetectorParameters(float leafSize, float clusterDist, int minClusterSize)
{
    m_pObjectDetector->setDownsamplingSize(leafSize);
    m_pObjectDetector->setClusteringIntradistanceFactor(clusterDist);
    m_pObjectDetector->setMinClusterSize(minClusterSize);
}

void ReMedi::setObjectRecognizerParameters(vector<ObjectModel<ColorPointT>::Ptr> objectModels, vector<float> objectRejections, int descriptionType, int strategy)
{
    m_DescriptionType = descriptionType;

    // Delete previously initialized object recognizer
    
    if (m_pObjectRecognizer != NULL)
    {
        if (m_DescriptionType == DESCRIPTION_FPFH)
            delete ((ObjectRecognizer<ColorPointT,FPFHSignature>*) m_pObjectRecognizer);
        else if (m_DescriptionType == DESCRIPTION_PFHRGB)
            delete ((ObjectRecognizer<ColorPointT,PFHRGBSignature>*) m_pObjectRecognizer);

        m_pObjectRecognizer = NULL;
    }
    
    // Initialize a new recognizer
    
    if (m_DescriptionType == DESCRIPTION_FPFH)
    {
        ObjectRecognizer<ColorPointT,FPFHSignature>* pObjectRecognizer = new ObjectRecognizer<ColorPointT,FPFHSignature>();
        pObjectRecognizer->setInputObjectModels(objectModels);
        pObjectRecognizer->setInputObjectRejections(objectRejections);
        pObjectRecognizer->setRecognitionStrategy(strategy);
        m_pObjectRecognizer = pObjectRecognizer;
    }
    else if (m_DescriptionType == DESCRIPTION_PFHRGB)
    {
        ObjectRecognizer<ColorPointT,PFHRGBSignature>* pObjectRecognizer = new ObjectRecognizer<ColorPointT,PFHRGBSignature>();
        pObjectRecognizer->setInputObjectModels(objectModels);
        pObjectRecognizer->setInputObjectRejections(objectRejections);
        pObjectRecognizer->setRecognitionStrategy(strategy);
        m_pObjectRecognizer = pObjectRecognizer;
    }
}

void ReMedi::setObjectRecognizerPfhParameters(float leafSize, float leafSizeModel, float normalRadius, float normalRadiusModel, float pfhRadius, float pfhRadiusModel, float ptScrRjtThresh)
{
    if (m_pObjectRecognizer != NULL)
    {
        if (m_DescriptionType == DESCRIPTION_FPFH)
        {
            ObjectRecognizer<ColorPointT,FPFHSignature>* pObjectRecognizer = (ObjectRecognizer<ColorPointT,FPFHSignature>*) m_pObjectRecognizer;
            pObjectRecognizer->setCloudjectsLeafSize(leafSize);
            pObjectRecognizer->setCloudjectModelsLeafSize(leafSizeModel);
            pObjectRecognizer->setCloudjectsNormalRadius(normalRadius);
            pObjectRecognizer->setCloudjectModelsNormalRadius(normalRadiusModel);
            pObjectRecognizer->setCloudjectsPfhRadius(pfhRadius);
            pObjectRecognizer->setCloudjectModelsPfhRadius(pfhRadiusModel);
            pObjectRecognizer->setPointScoreRejectionThreshold(ptScrRjtThresh);
        }
        else if (m_DescriptionType == DESCRIPTION_PFHRGB)
        {
            ObjectRecognizer<ColorPointT,PFHRGBSignature>* pObjectRecognizer = (ObjectRecognizer<ColorPointT,PFHRGBSignature>*) m_pObjectRecognizer;
            pObjectRecognizer->setCloudjectsLeafSize(leafSize);
            pObjectRecognizer->setCloudjectModelsLeafSize(leafSizeModel);
            pObjectRecognizer->setCloudjectsNormalRadius(normalRadius);
            pObjectRecognizer->setCloudjectModelsNormalRadius(normalRadiusModel);
            pObjectRecognizer->setCloudjectsPfhRadius(pfhRadius);
            pObjectRecognizer->setCloudjectModelsPfhRadius(pfhRadiusModel);
            pObjectRecognizer->setPointScoreRejectionThreshold(ptScrRjtThresh);
        }
    }
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
    
    // Create the cloudjects
    if (m_pObjectRecognizer != NULL)
    {
        if (m_DescriptionType == DESCRIPTION_FPFH)
            ((ObjectRecognizer<ColorPointT,FPFHSignature>*) m_pObjectRecognizer)->create();
        else if (m_DescriptionType == DESCRIPTION_PFHRGB)
            ((ObjectRecognizer<ColorPointT,PFHRGBSignature>*) m_pObjectRecognizer)->create();
    }
}

void ReMedi::run()
{
    initialize();
}

void ReMedi::loadSequences(std::string parent, std::vector<std::string>& names, std::vector<int>& indices)
{
    names.clear();
    indices.clear();
    
    const char* path = parent.c_str();
	if( boost::filesystem::exists( path ) )
	{
		boost::filesystem::directory_iterator end;
		boost::filesystem::directory_iterator iter(path);
        int i = -1;
        std::string sid = "";
		for( ; iter != end ; ++iter )
		{
			if ( boost::filesystem::is_directory( *iter ) )
            {
                std::string nameStr = iter->path().stem().string();
                names.push_back(nameStr);
                
                std::vector<string> nameStrL;
                boost::split(nameStrL, nameStr, boost::is_any_of("_"));
                if (nameStrL[1] != "bs")
                {
                    if (nameStrL[1] != sid)
                    {
                        i++;
                        sid = nameStrL[1];
                    }
                    indices.push_back(i);
                }
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

void ReMedi::loadObjectModels(const char* path, const char* pcdDir, vector<string> modelsNames,
                     vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >& views)
{
    pcl::PCDReader reader; // to read pcd files containing point clouds
    
    views.resize(modelsNames.size());
    for (int i = 0; i < modelsNames.size(); i++)
    {
        string objectPath = path + modelsNames[i] + "/" + pcdDir;
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> objectViews;
        
        if( boost::filesystem::exists( objectPath ) )
        {
            boost::filesystem::directory_iterator end;
            boost::filesystem::directory_iterator iter(objectPath);
            
            for( ; iter != end ; ++iter )
            {
                if ( !boost::filesystem::is_directory( *iter ) && iter->path().extension().string().compare(".pcd") == 0)
                {
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB>);
                    reader.read( iter->path().string(), *object );
                    
                    objectViews.push_back(object);
                }
            }
        }
        
        views[i] = objectViews;
    }
}

//void ReMedi::loadObjectsModels(string parent, vector<int>& objectsIDs, vector<string>& objectsNames, vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >& objectsViews)
//{
//    objectsIDs.clear();
//    objectsNames.clear();
//    objectsViews.clear();
//    
//    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> prevObjViews;
//    int prevObjID = 0;
//    string prevObjName = "";
//    
//    const char* c_parent = parent.c_str();
//	if( boost::filesystem::exists( c_parent ) )
//	{
//		boost::filesystem::directory_iterator end;
//		boost::filesystem::directory_iterator iter( c_parent );
//		for( ; iter != end ; ++iter )
//		{
//			if ( !boost::filesystem::is_directory( *iter ) )
//            {
//                // Determine id and object name from cloud file's filename
//                string filenameStemStr = iter->path().stem().string();
//                
//                if (filenameStemStr.size() > 0) // not .<filename> (hidden file in osx)
//                {
//                    vector<string> filenameSubwords;
//                    boost::split(filenameSubwords, filenameStemStr, boost::is_any_of("_"));
//                    
//                    int objID = stoi(filenameSubwords[0]);
//                    string objName = filenameSubwords[1];
//                    
//                    // Construct the cloudject using the views
//                    if ( ((prevObjID != objID) && (prevObjID > 0)))
//                    {
//                        objectsIDs.push_back(prevObjID);
//                        objectsNames.push_back(prevObjName);
//                        objectsViews.push_back(prevObjViews);
//                        
//                        prevObjViews.clear();
//                    }
//                    
//                    // Read point cloud
//                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
//                    
//                    pcl::PCDReader reader;
//                    reader.read(iter->path().string(), *pCloud);
//                    prevObjViews.push_back(pCloud);
//                    
//                    prevObjID = objID;
//                    prevObjName = objName;
//                }
//            }
//        }
//        
//        // Construct the cloudject using the views
//        if (prevObjViews.size() > 0)
//        {
//            objectsIDs.push_back(prevObjID);
//            objectsNames.push_back(prevObjName);
//            objectsViews.push_back(prevObjViews);
//        }
//    }
//}

