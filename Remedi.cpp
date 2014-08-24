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
  m_pTableModeler(new TableModeler)
{
#ifdef BS_USE_MOG2
    m_pBackgroundSubtractor = BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr(new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
#else
    m_pBackgroundSubtractor = BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>::Ptr(new BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>);
#endif
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
void ReMedi::setSubtractorParameters(int n, int m, int k, float lrate, float bgratio, float vargen, int level)
{
    m_pBackgroundSubtractor->setFramesResolution(m_XRes, m_YRes);
    
    // If models have not to be computed, we save the background seq loading time
    m_pBackgroundSubtractor->setBackgroundSequence(m_pBgSeq, n);
    
    m_pBackgroundSubtractor->setModality(m);
    m_pBackgroundSubtractor->setNumOfMixtureComponents(k);
    m_pBackgroundSubtractor->setLearningRate(lrate);
    m_pBackgroundSubtractor->setBackgroundRatio(bgratio);
    m_pBackgroundSubtractor->setVarThresholdGen(vargen);
    m_pBackgroundSubtractor->setOpeningSize(level);
}
#else
/** \brief Set the parameters of the background subtractor
 *  \param n The number of samples used for modelling
 *  \param m The modality used to model the background (color, color with shadows, or depth)
 *  \param f The maximum number of components per mixture in each pixel
 *  \param lrate The learning rate for the adaptive background model
 *  \param q The background ratio
 *  \param t The decision threshold based on the variance criterion
 *  \param level The size (2*level+1) of the kernel convolved to perform mathematical morphology (opening) to the background mask
 */
void ReMedi::setSubtractorParameters(int n, int m, int f, float lrate, float q, float t, int level)
{
    m_pBackgroundSubtractor->setFramesResolution(m_XRes, m_YRes);
    
    // If models have not to be computed, we save the background seq loading time
    m_pBackgroundSubtractor->setBackgroundSequence(m_pBgSeq, n);
    
    m_pBackgroundSubtractor->setModality(m);
    m_pBackgroundSubtractor->setNumOfMaxFeatures(f);
    m_pBackgroundSubtractor->setLearningRate(lrate);
    m_pBackgroundSubtractor->setQuantizationLevels(q);
    m_pBackgroundSubtractor->setDecisionThreshold(t);
    m_pBackgroundSubtractor->setOpeningSize(level);
}
#endif // BS_USE_MOG2

// Set the parameters of the monitorizer
//    leafsz : leaf size in the voxel grid downsampling (speeds up further processes)
//    clusterDist : distance threshold in spatial clustering of 3-D blobs
void ReMedi::setMonitorizerParameters(float leafsz, float clusterDist)
{
    
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

//void ReMedi::validateBS(vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences)
//{
//    initialize();
//    
//    vector<ColorDepthFrame::Ptr> frames;
//    m_pRegisterer->setInputFrames( m_pSequences[0]->getFrames(5) );
//    m_pRegisterer->registrate(frames);
//    
//    vector<Frame::Ptr> gtMasks = fgMasksSequences[0]->getFrames(5);
//    
//    // TODO: fer q les masqueres siguin tingudes en compte automaticament a subtract()
//    vector<cv::Mat> tabletopMasks;
//    m_pTableModeler->getTabletopMask(frames, tabletopMasks);
//    
//    vector< vector<double> > combinations;
//    expandParameters<double>(parameters, combinations);
//    
//    cv::Mat overlaps (combinations.size(), m_pSequences[0]->getNumOfViews(), cv::DataType<float>::type, cv::Scalar(0));
//    vector<vector<cv::Mat> > masks (combinations.size());
//    
//#ifdef BS_USE_MOG2
//    string dir = "bs-mog2_results/";
//#else
//    string dir = "bs-gmg_results/";
//#endif
//    
//    for (int p = 0; p < combinations.size(); p++)
//    {
//        for (int i = 0; i < combinations[p].size(); i++)
//            cout << combinations[p][i] << " ";
//        cout << endl;
//        
//        m_pBackgroundSubtractor->setModality(combinations[p][0]);
//        
//#ifdef BS_USE_MOG2
//        m_pBackgroundSubtractor->setNumOfMixtureComponents(combinations[p][1]);
//        m_pBackgroundSubtractor->setLearningRate(combinations[p][2]);
//        m_pBackgroundSubtractor->setBackgroundRatio(combinations[p][3]);
//        m_pBackgroundSubtractor->setVarThresholdGen(combinations[p][4]);
//        m_pBackgroundSubtractor->setOpeningSize(combinations[p][5]);
//#else
//        m_pBackgroundSubtractor->setNumOfMaxFeatures(combinations[p][1]);
//        m_pBackgroundSubtractor->setLearningRate(combinations[p][2]);
//        m_pBackgroundSubtractor->setQuantizationLevels(combinations[p][3]);
//        m_pBackgroundSubtractor->setDecisionThreshold(combinations[p][4]);
//        m_pBackgroundSubtractor->setOpeningSize(combinations[p][5]);
//#endif
//        // Train with background sequence
//        //        string dirname = m_pBackgroundSubtractor->getModality() % 2 == 0 ? "GMMs_Color" : "GMMs_Depth";
//        //m_pBackgroundSubtractor->model(dirname);
//        
//        m_pBackgroundSubtractor->model();
//        
//        
//        // Test with rest
//        m_pBackgroundSubtractor->setInputFrames(frames);
//        m_pBackgroundSubtractor->subtract(frames);
//        
//        vector<cv::Mat> predMasks (frames.size());
//        
//        for (int v = 0; v < frames.size(); v++)
//        {
//            int modality = m_pBackgroundSubtractor->getModality();
//            if ( (modality == ReMedi::COLOR) || (modality == ReMedi::COLOR_WITH_SHADOWS) )
//                predMasks[v] = frames[v]->getColorMask();
//            else if (modality == ReMedi::DEPTH)
//                predMasks[v] = frames[v]->getDepthMask();
//            else if (modality == ReMedi::COLORDEPTH)
//                predMasks[v] = frames[v]->getMask();
//            
//            cv::bitwise_and(predMasks[v], tabletopMasks[v], predMasks[v]);
//            overlaps.at<float>(p,v) += cvx::overlap(predMasks[v], gtMasks[v]->get());
//            
//            string s = dir + to_string(v) + "_" + to_string(p) + "-";
//            for (int i = 0; i < combinations[p].size(); i++)
//                s += to_string_with_precision( combinations[p][i], 2) + "_";
//            s += to_string(overlaps.at<float>(p,v));
//            cv::imwrite(s + ".png", predMasks[v]); // qualitative
//        }
//        
//        cout << overlaps.row(p) << " ";
//        masks[p] = predMasks;
//        
//        cout << endl;
//    }
//    
//    //
//    // Illustrate best results (based in overlap measure)
//    //
//    
//    cv::Mat I;
//    cv::sortIdx(overlaps, I, cv::SORT_EVERY_COLUMN | cv::SORT_DESCENDING);
//    
//    for (int p = 0; p < I.rows; p++)
//    {
//        for (int v = 0; v < I.cols; v++)
//        {
//            string s = dir + to_string(v) + "_S" + to_string(p) + "_(";
//            int idx = I.at<int>(p,v);
//            s += to_string(idx) + ")-";
//            
//            for (int i = 0; i < combinations[idx].size(); i++)
//                s += to_string_with_precision( combinations[idx][i], 2) + "_";
//            
//            float ovl = overlaps.at<float>(I.at<int>(p,v), v);
//            s += to_string(ovl);
//            
//            cout << s << endl; // quantitative
//            cv::imwrite(s + ".png", masks[idx][v]); // qualitative
//        }
//    }
//}

void reloadExistingOverlapsFile(string path, cv::Mat& combinations, cv::Mat& overlaps)
{
    combinations.release();
    overlaps.release();
    
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    
    cv::Mat combinationsLine, overlapsLine;
    int i = 0;
    do
    {
        fs["combination_" + to_string(i)] >> combinationsLine;
        fs["overlaps_" + to_string(i)]    >> overlapsLine;
        
        if (!combinationsLine.empty() && !overlapsLine.empty())
        {
            if (combinations.empty()) combinations = combinationsLine.clone();
            else cv::vconcat(combinations, combinationsLine, combinations);
            
            if (overlaps.empty()) overlaps = overlapsLine.clone();
            else cv::vconcat(overlaps, overlapsLine, overlaps);
        }
        
        i++;
    }
    while (!combinationsLine.empty());
    
    fs.release();
}

void ReMedi::validateBS(vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences)
{
    initialize();
    
    cv::Mat combinations;
    expandParameters<double>(parameters, combinations);
    
    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = m_pBgSeq->getNumOfViews();
    int numOfFrames = fgMasksSequences[0]->getNumOfFrames()[0];
    
    for (int s = 0; s < fgMasksSequences.size(); s++)
    {
        assert (numOfViews == fgMasksSequences[s]->getNumOfViews());
        for (int v = 0; v < numOfViews; v++)
            assert (numOfFrames == fgMasksSequences[s]->getNumOfFrames()[v]);
    }
    // -------------------------------------------------

    string parentDir;
#ifdef BS_USE_MOG2
    parentDir = "bs-mog2_results/";
#else
    parentDir = "bs-gmg_results/";
#endif
    
    cv::Mat overlaps (combinations.rows, numOfViews * fgMasksSequences.size() * numOfFrames, cv::DataType<float>::type, cv::Scalar(0));
    vector<bool> revalidate (combinations.rows, true);
    
    // Temporal file already existing? backup it!
    boost::filesystem::path filePath (parentDir + "validation.tmp.yml");
    if (boost::filesystem::exists(filePath))
    {
        boost::filesystem::path bakFilePath (parentDir + "validation.tmp.bak.yml");
        cout << "Backup of " + filePath.string() + " to " + bakFilePath.string() + " ... " << endl;

        boost::filesystem::remove(bakFilePath); // get rid of prev backup first
        boost::filesystem::copy(filePath, bakFilePath); // create a backup
        
        cv::Mat combinationsOld, overlapsOld;
        reloadExistingOverlapsFile(bakFilePath.string(), combinationsOld, overlapsOld);
        
        for (int c = 0; c < combinations.rows; c++)
        {
            // Has a certain combination been previously computed?
            int idx = cvx::match<double>(combinationsOld, combinations.row(c), c == 0); // invokation: (db, query)
            if (idx >= 0)
            {
                revalidate[c] = false; // no need to revalidate
                overlapsOld.row(idx).copyTo(overlaps.row(c));
            }
        }
    }
    
    cv::FileStorage fs;
    boost::timer t;

    for (int c = 0; c < combinations.rows; c++)
    {
        t.restart();
        fs.open(parentDir + "validation.tmp.yml", cv::FileStorage::APPEND);
        
        string parametersStr = to_string_with_precision(combinations.at<double>(c,0), 4);
        for (int p = 1; p < combinations.cols; p++)
            parametersStr += "_" + to_string_with_precision(combinations.at<double>(c,p), 4);
        cout << c << " : " << parametersStr << endl;
        
        if (revalidate[c])
        {
            m_pBackgroundSubtractor->setModality(combinations.at<double>(c,0));
#ifdef BS_USE_MOG2
            m_pBackgroundSubtractor->setNumOfMixtureComponents(combinations.at<double>(c,1));
            m_pBackgroundSubtractor->setLearningRate(combinations.at<double>(c,2));
            m_pBackgroundSubtractor->setBackgroundRatio(combinations.at<double>(c,3));
            m_pBackgroundSubtractor->setVarThresholdGen(combinations.at<double>(c,4));
            m_pBackgroundSubtractor->setOpeningSize(combinations.at<double>(c,5));
#else
            m_pBackgroundSubtractor->setNumOfMaxFeatures(combinations.at<double>(c,1));
            m_pBackgroundSubtractor->setLearningRate(combinations.at<double>(c,2));
            m_pBackgroundSubtractor->setQuantizationLevels(combinations.at<double>(c,3));
            m_pBackgroundSubtractor->setDecisionThreshold(combinations.at<double>(c,4));
            m_pBackgroundSubtractor->setOpeningSize(combinations.at<double>(c,5));
#endif

            m_pBackgroundSubtractor->model();
        
            for (int s = 0; s < fgMasksSequences.size(); s++)
            {
                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
                Sequence<ColorDepthFrame>::Ptr pTestSeq = m_pSequences[s];
                
                string seqPath = parentDir + to_string(s+1) + "/";
                
                pFgGtSeq->restart();
                while (pFgGtSeq->hasNextFrames())
                {
                    vector<Frame::Ptr> fgGtMasksFrames = pFgGtSeq->nextFrames();
                    
                    // search for correspondence using filename
                    vector<string> filenames = pFgGtSeq->getFramesFilenames();
                    vector<ColorDepthFrame::Ptr> testFrames = pTestSeq->getFrames(filenames);
                    m_pRegisterer->setInputFrames(testFrames);
                    m_pRegisterer->registrate(testFrames);
                    
                    // focus only on tabletop's region for the BS
                    vector<cv::Mat> tabletopMasks;
                    m_pTableModeler->getTabletopMask(testFrames, tabletopMasks);
                    
                    // test
                    m_pBackgroundSubtractor->setInputFrames(testFrames);
                    m_pBackgroundSubtractor->subtract(testFrames);
                    
                    // gather the test results
                    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
                    {
                        cv::Mat prediction;
                        if ( (combinations.at<double>(c,0) == ReMedi::COLOR) || (combinations.at<double>(c,0) == ReMedi::COLOR_WITH_SHADOWS) )
                            prediction = testFrames[v]->getColorMask();
                        else if (combinations.at<double>(c,0) == ReMedi::DEPTH)
                            prediction = testFrames[v]->getDepthMask();
                        else if (combinations.at<double>(c,0) == ReMedi::COLORDEPTH)
                            prediction = testFrames[v]->getMask();
                        
                        // Masks were in disk as RGB images (preprocessing)
                        cv::Mat fgGtGrayMask, fgGtBinMask;
                        cv::cvtColor(fgGtMasksFrames[v]->get(), fgGtGrayMask, CV_BGR2GRAY);
                        cv::threshold(fgGtGrayMask, fgGtBinMask, 0, 255, cv::THRESH_BINARY);
                        
                        cv::Mat maskedPrediction = prediction & tabletopMasks[v];

                        // Quantitative result (overlap)
                        int idx = (v * fgMasksSequences.size() * numOfFrames) + (s * numOfFrames) + pFgGtSeq->getFrameCounters()[v];
                        overlaps.at<float>(c,idx) = cvx::overlap(maskedPrediction, fgGtBinMask);
                    
                        // Qualitative result (mask image)
                        string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                        cv::imwrite(maskPath + filenames[v] + "-" + parametersStr + ".png", maskedPrediction);
                    }
                }
            }
        }
        
        fs << "combination_" + to_string(c) << combinations.row(c);
        fs << "overlaps_" + to_string(c) << overlaps.row(c);
        fs.release();
        
        cout << to_string_with_precision(t.elapsed(), 3) << " seconds elapsed." << endl;
    }
    
    // Write results to a file
    
    fs.open(parentDir + "validation.yml", cv::FileStorage::WRITE);
    fs << "combinations" << combinations;
    fs << "overlaps" << overlaps;
    fs.release();
}

void ReMedi::showBsParametersPerformance(vector<Sequence<Frame>::Ptr> fgMasksSequences, string filePath)
{
    cv::Mat combinations, overlaps;
    
    string parentDir;
#ifdef BS_USE_MOG2
    parentDir = "bs-mog2_results/";
#else
    parentDir = "bs-gmg_results/";
#endif
    
    cv::FileStorage fs;
    fs.open(parentDir + filePath, cv::FileStorage::READ);
    fs["combinations"] >> combinations;
    fs["overlaps"] >> overlaps;
    fs.release();
    
    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
    {
        cv::Rect roi (v * numOfFrames, 0, numOfFrames, combinations.rows);
        cv::Mat overlapsView (overlaps, roi);
        
        cv::Mat overlapAvgView;
        cv::reduce(overlapsView, overlapAvgView, 1, CV_REDUCE_AVG);
    
        cv::Mat I;
        cv::sortIdx(overlapAvgView, I, cv::SORT_EVERY_COLUMN | cv::SORT_DESCENDING);
        
        int best = 10;
        for (int i = 0; i < best; i++)
        {
            int idx = I.at<int>(0,i);
            cout << i << "/" << best << combinations.row(idx) << "\t" << overlapAvgView.row(idx) << endl;
            
            for (int s = 3; s < 4 /*fgMasksSequences.size()*/; s++)
            {
                string seqPath = parentDir + to_string(s+1) + "/";
                string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                
                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
                pFgGtSeq->restart();
                //while (pFgGtSeq->hasNextFrames())
                //{
                    vector<Frame::Ptr> frames = pFgGtSeq->nextFrames();
                    vector<string> filenames = pFgGtSeq->getFramesFilenames();
                    
                    cout << v << " " << i << " " << s << " " << filenames[v] << endl;
                    
                    string filename = filenames[v] + "-"; // then concantenated with params combination
                    filename += to_string_with_precision(combinations.at<double>(idx,0), 4);
                    for (int p = 1; p < combinations.cols; p++)
                        filename += "_" + to_string_with_precision(combinations.at<double>(idx,p), 4);

                    cv::Mat maskedPrediction;
                    maskedPrediction = cv::imread(maskPath + filename + ".png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_UNCHANGED);
                    
                    cv::imshow("loaded", maskedPrediction);
                    cv::waitKey(0);
                //}
            }
        }
        cout << endl;
    }
    cout << endl;
}

void ReMedi::getBsParametersPerformance(string filePath, vector<vector<double> >& combinationsSorted)
{
    cv::Mat combinations, overlaps;
    
    string parentDir;
#ifdef BS_USE_MOG2
    parentDir = "bs-mog2_results/";
#else
    parentDir = "bs-gmg_results/";
#endif
    
    cv::FileStorage fs;
    fs.open(parentDir + filePath, cv::FileStorage::READ);
    fs["combinations"] >> combinations;
    fs["overlaps"] >> overlaps;
    fs.release();
    
    cv::Mat means (overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
    cv::Mat stddevs (overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
    
    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
    {
        cv::Rect roi (v * numOfFrames, 0, numOfFrames, combinations.rows);
        cv::Mat overlapsView (overlaps, roi);
        
        for (int i = 0; i < overlaps.rows; i++)
        {
            cv::Scalar m, s;
            cv::meanStdDev(overlapsView.row(i), m, s);
            means.at<float>(i,v) = m.val[0];
            stddevs.at<float>(i,v) = s.val[0];
        }
    }
    
    cv::Mat meansAvg;
    cvx::harmonicMean(means, meansAvg, 1);
    
    cv::Mat I, meansAvgSorted;
    cvx::sortIdx(meansAvg, I, CV_SORT_EVERY_COLUMN | CV_SORT_DESCENDING, meansAvgSorted);
    
    cv::Mat _combinationsSorted = cvx::indexMat(combinations, I, false);
    
    // conversion from cv::Mat to STL structure
    cvx::convert<double>(_combinationsSorted, combinationsSorted);
}

void ReMedi::validateBS(vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences)
{
    initialize();
    
    cv::Mat combinations;
    expandParameters<double>(parameters, combinations);
    
    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = m_pBgSeq->getNumOfViews();
    int numOfFrames = fgMasksSequences[0]->getNumOfFrames()[0];
    
    for (int s = 0; s < fgMasksSequences.size(); s++)
    {
        assert (numOfViews == fgMasksSequences[s]->getNumOfViews());
        for (int v = 0; v < numOfViews; v++)
            assert (numOfFrames == fgMasksSequences[s]->getNumOfFrames()[v]);
    }
    // -------------------------------------------------
    
    string parentDir;
#ifdef BS_USE_MOG2
    parentDir = "bs-mog2_results/";
#else
    parentDir = "bs-gmg_results/";
#endif
    
    cv::Mat overlaps (combinations.rows, numOfViews * fgMasksSequences.size() * numOfFrames, cv::DataType<float>::type, cv::Scalar(0));
    vector<bool> revalidate (combinations.rows, true);
    
    // Temporal file already existing? backup it!
    boost::filesystem::path filePath (parentDir + "validation.tmp.yml");
    if (boost::filesystem::exists(filePath))
    {
        boost::filesystem::path bakFilePath (parentDir + "validation.tmp.bak.yml");
        cout << "Backup of " + filePath.string() + " to " + bakFilePath.string() + " ... " << endl;
        
        boost::filesystem::remove(bakFilePath); // get rid of prev backup first
        boost::filesystem::copy(filePath, bakFilePath); // create a backup
        
        cv::Mat combinationsOld, overlapsOld;
        reloadExistingOverlapsFile(bakFilePath.string(), combinationsOld, overlapsOld);
        
        for (int c = 0; c < combinations.rows; c++)
        {
            // Has a certain combination been previously computed?
            int idx = cvx::match<double>(combinationsOld, combinations.row(c), c == 0); // invokation: (db, query)
            if (idx >= 0)
            {
                revalidate[c] = false; // no need to revalidate
                overlapsOld.row(idx).copyTo(overlaps.row(c));
            }
        }
    }
    
    cv::FileStorage fs;
    boost::timer t;
    
    for (int c = 0; c < combinations.rows; c++)
    {
        t.restart();
        fs.open(parentDir + "validation.tmp.yml", cv::FileStorage::APPEND);
        
        string parametersStr = to_string_with_precision(combinations.at<double>(c,0), 4);
        for (int p = 1; p < combinations.cols; p++)
            parametersStr += "_" + to_string_with_precision(combinations.at<double>(c,p), 4);
        cout << c << " : " << parametersStr << endl;
        
        if (revalidate[c])
        {
            m_pBackgroundSubtractor->setModality(combinations.at<double>(c,0));
#ifdef BS_USE_MOG2
            m_pBackgroundSubtractor->setNumOfMixtureComponents(combinations.at<double>(c,1));
            m_pBackgroundSubtractor->setLearningRate(combinations.at<double>(c,2));
            m_pBackgroundSubtractor->setBackgroundRatio(combinations.at<double>(c,3));
            m_pBackgroundSubtractor->setVarThresholdGen(combinations.at<double>(c,4));
            m_pBackgroundSubtractor->setOpeningSize(combinations.at<double>(c,5));
#else
            m_pBackgroundSubtractor->setNumOfMaxFeatures(combinations.at<double>(c,1));
            m_pBackgroundSubtractor->setLearningRate(combinations.at<double>(c,2));
            m_pBackgroundSubtractor->setQuantizationLevels(combinations.at<double>(c,3));
            m_pBackgroundSubtractor->setDecisionThreshold(combinations.at<double>(c,4));
            m_pBackgroundSubtractor->setOpeningSize(combinations.at<double>(c,5));
#endif
            
            m_pBackgroundSubtractor->model();
            
            for (int s = 0; s < fgMasksSequences.size(); s++)
            {
                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
                Sequence<ColorDepthFrame>::Ptr pTestSeq = m_pSequences[s];
                
                string seqPath = parentDir + to_string(s+1) + "/";
                
                pFgGtSeq->restart();
                while (pFgGtSeq->hasNextFrames())
                {
                    vector<Frame::Ptr> fgGtMasksFrames = pFgGtSeq->nextFrames();
                    
                    // search for correspondence using filename
                    vector<string> filenames = pFgGtSeq->getFramesFilenames();
                    vector<ColorDepthFrame::Ptr> testFrames = pTestSeq->getFrames(filenames);
                    m_pRegisterer->setInputFrames(testFrames);
                    m_pRegisterer->registrate(testFrames);
                    
                    // focus only on tabletop's region for the BS
                    vector<cv::Mat> tabletopMasks;
                    m_pTableModeler->getTabletopMask(testFrames, tabletopMasks);
                    
                    // test
                    m_pBackgroundSubtractor->setInputFrames(testFrames);
                    m_pBackgroundSubtractor->subtract(testFrames);
                    
                    // gather the test results
                    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
                    {
                        cv::Mat prediction;
                        if ( (combinations.at<double>(c,0) == ReMedi::COLOR) || (combinations.at<double>(c,0) == ReMedi::COLOR_WITH_SHADOWS) )
                            prediction = testFrames[v]->getColorMask();
                        else if (combinations.at<double>(c,0) == ReMedi::DEPTH)
                            prediction = testFrames[v]->getDepthMask();
                        else if (combinations.at<double>(c,0) == ReMedi::COLORDEPTH)
                            prediction = testFrames[v]->getMask();
                        
                        // Masks were in disk as RGB images (preprocessing)
                        cv::Mat fgGtGrayMask, fgGtBinMask;
                        cv::cvtColor(fgGtMasksFrames[v]->get(), fgGtGrayMask, CV_BGR2GRAY);
                        cv::threshold(fgGtGrayMask, fgGtBinMask, 0, 255, cv::THRESH_BINARY);
                        
                        cv::Mat maskedPrediction = prediction & tabletopMasks[v];
                        
                        // Quantitative result (overlap)
                        int idx = (v * fgMasksSequences.size() * numOfFrames) + (s * numOfFrames) + pFgGtSeq->getFrameCounters()[v];
                        overlaps.at<float>(c,idx) = cvx::overlap(maskedPrediction, fgGtBinMask);
                        
                        // Qualitative result (mask image)
                        string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                        cv::imwrite(maskPath + filenames[v] + "-" + parametersStr + ".png", maskedPrediction);
                    }
                }
            }
        }
        
        fs << "combination_" + to_string(c) << combinations.row(c);
        fs << "overlaps_" + to_string(c) << overlaps.row(c);
        fs.release();
        
        cout << to_string_with_precision(t.elapsed(), 3) << " seconds elapsed." << endl;
    }
    
    // Write results to a file
    
    fs.open(parentDir + "validation.yml", cv::FileStorage::WRITE);
    fs << "combinations" << combinations;
    fs << "overlaps" << overlaps;
    fs.release();
}

void ReMedi::validateMonitorizerClustering(vector<Sequence<Frame>::Ptr> fgMasksSequences, vector<vector<double> > bsCombinations)
{
    initialize();

    string parentDir;
#ifdef BS_USE_MOG2
    parentDir = "bs-mog2_results/";
#else
    parentDir = "bs-gmg_results/";
#endif
    
    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
    {
        for (int i = 0; i < bsCombinations.size(); i++)
        {
            for (int s = 0; s < fgMasksSequences.size(); s++)
            {
                string seqPath = parentDir + to_string(s+1) + "/";
                string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                
                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
                pFgGtSeq->restart();
                while (pFgGtSeq->hasNextFrames())
                {
                    vector<Frame::Ptr> frames = pFgGtSeq->nextFrames();
                    vector<string> filenames = pFgGtSeq->getFramesFilenames();
                    
                    cout << v << " " << i << " " << s << " " << filenames[v] << endl;
                    
                    string filename = filenames[v] + "-"; // then concantenated with params combination
                    filename += to_string_with_precision(bsCombinations[i][0], 4);
                    for (int p = 1; p < bsCombinations[i].size(); p++)
                        filename += "_" + to_string_with_precision(bsCombinations[i][p], 4);
                    
                    cv::Mat maskedPrediction = cv::imread(maskPath + filename + ".png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_UNCHANGED);
                    
                    cv::imshow("loaded", maskedPrediction);
                    cv::waitKey(0);
                }
            }
        }
    }
}
