 //
//  recognition_validation.cpp
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#include "recognition_validation.h"
#include "segmentation_validation.h"
#include <boost/thread.hpp>
#include <iostream>     // std::cout
#include <iterator>     // std::ostream_iterator

boost::mutex g_Mutex;

//
// ScoredDetections class
//

ScoredDetections::ScoredDetections()
{
    
}

ScoredDetections::ScoredDetections(std::vector<std::vector<int> > vids, std::vector<std::vector<pcl::PointXYZ> > positions, std::vector<std::vector< std::vector<float> > > scores)
: m_Vids(vids), m_Positions(positions), m_Scores(scores)
{
    
}

ScoredDetections::ScoredDetections(cv::Mat positions, cv::Mat scores)
{
    setSparseRepresentation(positions, scores);
}

ScoredDetections::ScoredDetections(const ScoredDetections& rhs)
{
    *this = rhs;
}

ScoredDetections& ScoredDetections::operator=(const ScoredDetections& rhs)
{
    if (this != &rhs)
    {
        m_Vids = rhs.m_Vids;
        m_Positions = rhs.m_Positions;
        m_Scores = rhs.m_Scores;
    }
    
    return *this;
}

void ScoredDetections::set(std::vector<std::vector<int> > vids, std::vector<std::vector<pcl::PointXYZ> > positions, std::vector<std::vector< std::vector<float> > > scores)
{
    m_Vids = vids;
    m_Positions = positions;
    m_Scores = scores;
}

void ScoredDetections::get(std::vector<std::vector<int> >& vids, std::vector<std::vector<pcl::PointXYZ> >& positions, std::vector<std::vector< std::vector<float> > >& scores)
{
    vids = m_Vids;
    positions = m_Positions;
    scores = m_Scores;
}

std::vector<std::vector<int> > ScoredDetections::getVids()
{
    return m_Vids;
}

std::vector<std::vector< pcl::PointXYZ > > ScoredDetections::getPositions()
{
    return m_Positions;
}

std::vector<std::vector< std::vector<float> > > ScoredDetections::getScores()
{
    return m_Scores;
}

bool ScoredDetections::empty()
{
    return m_Positions.empty();
}

//void ScoredDetections::append(std::string filePath, std::string name)
//{
//    cv::Mat positionsSparse, scoresSparse;
//    getSparseRepresentation(positionsSparse, scoresSparse);
//    
//    cv::FileStorage fs (filePath, cv::FileStorage::APPEND);
//    fs << ("positions_" + name) << positionsSparse;
//    fs << ("scores_" + name) << scoresSparse;
//    fs.release();
//}
//
//void ScoredDetections::load(std::string filePath, std::string name)
//{
//    cv::Mat positionsSparse, scoresSparse;
//    
//    cv::FileStorage fs (filePath, cv::FileStorage::READ);
//    fs["positions_" + name] >> positionsSparse;
//    fs["scores_" + name] >> scoresSparse;
//    fs.release();
//    
//    setSparseRepresentation(positionsSparse, scoresSparse);
//}

void ScoredDetections::setSparseRepresentation(cv::Mat positions, cv::Mat scores)
{
    m_Vids.resize(positions.rows);
    m_Positions.resize(positions.rows);
    m_Scores.resize(positions.rows);
    
    for (int i = 0; i < positions.rows; i++)
    {
        for (int j = 0; j < positions.cols; j++)
        {
            cv::Vec3f pos = positions.at<cv::Vec3f>(i,j);
            if ( !(pos[0] == 0 && pos[1] == 0 && pos[2] == 0) )
            {
                // vids
                m_Vids[i].push_back(j);
                // positions
                m_Positions[i].push_back( pcl::PointXYZ(pos[0], pos[1], pos[2]) );
                // scores
                cv::Mat roi (scores, cv::Rect(j*OD_NUM_OF_OBJECTS, i, OD_NUM_OF_OBJECTS, 1));
                m_Scores[i].push_back( std::vector<float>(roi.begin<float>(), roi.end<float>()) );
            }
        }
    }
}

void ScoredDetections::getSparseRepresentation(cv::Mat& positionsSparse, cv::Mat& scoresSparse)
{
    positionsSparse.create(m_Positions.size(), NUM_OF_VIEWS, CV_32FC(3));
    scoresSparse.create(m_Scores.size(), NUM_OF_VIEWS * OD_NUM_OF_OBJECTS, CV_32FC1);
    
    positionsSparse.setTo(0);
    scoresSparse.setTo(0);
    
    for (int i = 0; i < m_Vids.size(); i++)
    {
        for (int j = 0; j < m_Vids[i].size(); j++)
        {
            int vid = m_Vids[i][j];
            
            pcl::PointXYZ p = m_Positions[i][j];
            positionsSparse.at<cv::Vec3f>(i,vid) = cv::Vec3f(p.x, p.y, p.z);
            
            for (int k = 0; k < OD_NUM_OF_OBJECTS; k++)
                scoresSparse.at<float>(i, vid * OD_NUM_OF_OBJECTS + k) = m_Scores[i][j][k];
        }
    }
}

//
//
//

//void loadPrecomputedRecognitionScoresFile(std::string filePath, std::vector<int> combinationsIndices, std::vector<int> sequencesIndices, vector<vector<vector<cv::Mat> > >& positions, vector<vector<vector<cv::Mat> > >& scores)
//{
//    combinations.release();
//    positions.clear();
//    scores.clear();
//    
//    cv::FileStorage fs;
//    fs.open(filePath, cv::FileStorage::READ);
//    
//    fs["bg-sgmt_combinations"] >> combinations;
//    
//    positions.resize(combinations.rows);
//    scores.resize(combinations.rows);
//    for (int i = 0; i < combinations.rows; i++)
//    {
//        std::vector<std::vector<cv::Mat> > positionsComb, scoresComb;
//        std::vector<cv::Mat> positionsSeq, scoresSeq;
//        int s = 0;
//        do
//        {
//            cv::Mat positionsFrame, scoresFrame;
//            int f = 0;
//            do
//            {
//                string id = to_str(i) + "-" + to_str(s) + "-" + to_str(f);
//                fs["positions_" + id] >> positionsFrame;
//                fs["scores_" + id] >> scoresFrame;
//                
//                if (!scoresFrame.empty())
//                {
//                    positionsSeq.push_back(positionsFrame);
//                    scoresSeq.push_back(scoresFrame);
//                }
//                f++;
//            }
//            while (!scoresFrame.empty());
//            
//            if (!scoresSeq.empty())
//            {
//                positionsComb.push_back(positionsSeq);
//                scoresComb.push_back(scoresSeq);
//            }
//            s++;
//        }
//        while (!scoresSeq.empty());
//        
//        positions[i] = positionsComb;
//        scores[i] = scoresComb;
//    }
//    
//    fs.release();
//}

void loadMonitorizationRecognitionPrecomputedScoresFile(std::string filePath, int cid, int sid, std::vector<ScoredDetections>& scoredsFrames)
{
    scoredsFrames.clear();
    
    cv::FileStorage fs (filePath, cv::FileStorage::READ);
    int f = 0;
    
    bool bSuccess = true;
    while (bSuccess)
    {
        std::string id = to_str(cid) + "-" + to_str(sid) + "-" + to_str(f);
        
        cv::Mat positionsSparse, scoresSparse;
        fs["positions_" + id] >> positionsSparse;
        fs["scores_" + id] >> scoresSparse;
        
        ScoredDetections s (positionsSparse, scoresSparse);
        
        if (s.empty()) bSuccess = false;
        else scoredsFrames.push_back(s);
        
        f++;
    };
    
    fs.release();
}

void loadMonitorizationRecognitionScoredDetections(std::string filePath, std::vector<int> combinationsIds, std::vector<int> sequencesIds, std::vector<std::vector<std::vector<ScoredDetections> > >& scoreds)
{
    scoreds.resize(combinationsIds.size());
    for (int i = 0; i < combinationsIds.size(); i++)
        scoreds[i].resize(sequencesIds.size());
    
    for (int i = 0; i < combinationsIds.size(); i++)
    {
        for (int j = 0; j < sequencesIds.size(); j++)
        {
            std::vector<cv::Mat> positionsSeq;
            std::vector<cv::Mat> scoresSeq;
            loadMonitorizationRecognitionPrecomputedScoresFile(filePath, combinationsIds[i], sequencesIds[j], scoreds[i][j]);
        }
    }
}

void perform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float r, float a, float b, float c, pcl::PointCloud<pcl::PointXYZRGB>& table_out)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_aux (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (0.005, 0.005, 0.005);
    vg.setDownsampleAllData (true);
    vg.filter (*cloud_in_aux);

    cloud_in_aux.swap(cloud_in);
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    // Set up a Normal Estimation class and merge data in cloud_with_normals
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (r);
    ne.compute (*normals);
    
    pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);
    
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB,pcl::Normal> sac (false);
	sac.setOptimizeCoefficients (true); // optional
    sac.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    sac.setMethodType (pcl::SAC_RANSAC);
    sac.setMaxIterations (a);
    sac.setDistanceThreshold (b);
    sac.setNormalDistanceWeight(c);
    
    // create filtering object
    
    pcl::ModelCoefficients::Ptr pCoefficients (new pcl::ModelCoefficients);
    
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> pointExtractor;
    pointExtractor.setKeepOrganized(true);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_plane (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_nonplane (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_filtered (new pcl::PointCloud<pcl::Normal>);
    
    int n = cloud_in->size();
    //	while (cloud_out->size() > 0.3 * n) // Do just one iteration!
    //    {
    // Segment the largest planar component from the remaining cloud
    sac.setInputCloud(cloud_in);
    sac.setInputNormals(normals);
    sac.segment(*inliers, *pCoefficients);
    
    // Extract the inliers (points in the plane)
    pointExtractor.setInputCloud (cloud_with_normals);
    pointExtractor.setIndices (inliers);
    pointExtractor.setNegative (false);
    pointExtractor.filter (*cloud_with_normals_plane);
    pointExtractor.setNegative(true);
    pointExtractor.filter(*cloud_with_normals_nonplane);
    
    pcl::copyPointCloud(*cloud_with_normals_nonplane, *cloud_in);
    pcl::copyPointCloud(*cloud_with_normals_nonplane, *normals);
    
    table_out = *cloud_in;
}

void _precomputeScores(ReMedi::Ptr pSys, vector<ColorDepthFrame::Ptr> frames, BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS, cv::Mat combination, int offset, std::string filePath, std::string id, ScoredDetections& scoreds, std::vector<std::vector<pcl::PointXYZ> >& detectionPositions)
{
    g_Mutex.lock();
    pSys->getRegisterer()->setInputFrames(frames);
    pSys->getRegisterer()->registrate(frames);
    g_Mutex.unlock();
    
    g_Mutex.lock();
    vector<cv::Mat> foregroundMasks;
    pBS->setInputFrames(frames);
    pBS->subtract(foregroundMasks);
    g_Mutex.unlock();
    
    for (int v = 0; v < frames.size(); v++)
    {
        frames[v]->setMask(foregroundMasks[v]);
    }
    
    vector<cv::Mat> tabletopMasks, interactionMasks;
    pSys->getTableModeler()->getTabletopMask(frames, tabletopMasks);
    pSys->getTableModeler()->getInteractionMask(frames, interactionMasks);
    
    ObjectDetector od (*pSys->getObjectDetector());
    od.setDownsamplingSize(combination.at<double>(0, offset + 0));
    od.setClusteringIntradistanceFactor(combination.at<double>(0, offset + 1));
    od.setMinClusterSize(combination.at<double>(0, offset + 2));
    od.setInterviewCorrepondenceDistance(combination.at<double>(0, offset + 3));
    
    od.setInputFrames(frames);
    od.setActorMasks(tabletopMasks);
    od.setInteractionMasks(interactionMasks);
    
    od.detect();
    
    detectionPositions.clear();
    od.getDetectionPositions(detectionPositions);
    
    vector<vector< pair<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > > detectionsCorrespondences;
    od.getDetectionCorrespondences(detectionsCorrespondences, true);
    
    vector<vector< int > > detectionsVids;
    vector<vector< pcl::PointXYZ > > _detectionsPositions;
    vector<vector< vector<float> > > detectionsScores;
    
    if (pSys->getDescriptionType() == DESCRIPTION_FPFH)
    {
        ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) pSys->getObjectRecognizer()) );
        orc.setInputDetections(detectionsCorrespondences);
        orc.getScores(detectionsVids, _detectionsPositions, detectionsScores);
    }
    else if (pSys->getDescriptionType() == DESCRIPTION_PFHRGB)
    {
        ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) pSys->getObjectRecognizer()) );
        orc.setInputDetections(detectionsCorrespondences);
        orc.getScores(detectionsVids, _detectionsPositions, detectionsScores);
    }
    
    // To proper format and save
    scoreds.set(detectionsVids, _detectionsPositions, detectionsScores);
   
    g_Mutex.lock();
    cv::Mat positionsSparse, scoresSparse;
    scoreds.getSparseRepresentation(positionsSparse, scoresSparse);
    cv::FileStorage fs (filePath, cv::FileStorage::APPEND);
    fs << ("positions_" + id) << positionsSparse;
    fs << ("scores_" + id) << scoresSparse;
    fs.release();
    g_Mutex.unlock();
}

void precomputeRecognitionScores(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsIndices, cv::Mat combinations, std::vector<int> combsIndices, std::vector<DetectionOutput> detectionGroundtruths, string path, string filename, vector<vector<vector<ScoredDetections> > >& scoreds, int numOfThreads)
{
    pSys->initialize();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = pSys->getBackgroundSequence();
    
    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = pBgSeq->getNumOfViews();
    for (int k = 0; k < seqsIndices.size(); k++)
        assert (numOfViews == sequences[seqsIndices[k]]->getNumOfViews());
    // -------------------------------------------------
    
    cv::Mat scoresCombinations, scoresIndices;
    cvx::unique(combinations, 0, scoresCombinations, scoresIndices);
    
    // Do not repeat the bs every time. Pre-compute the required combinations!
    cv::Mat bsCombinations, bsIndices;
    cvx::unique(scoresCombinations.colRange(0,6), 0, bsCombinations, bsIndices); // separate (bs,sgmn) parameters in combinations and find unique bs' sub-combinations
    
    vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors (bsCombinations.rows);
    
    for (int i = 0; i < bsCombinations.rows; i++)
    {
        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs (new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
        *pBs = *(pSys->getBackgroundSubtractor());
        
        pBs->setModality(bsCombinations.at<double>(i,0));
        pBs->setNumOfMixtureComponents(bsCombinations.at<double>(i,1));
        pBs->setLearningRate(bsCombinations.at<double>(i,2));
        pBs->setBackgroundRatio(bsCombinations.at<double>(i,3));
        pBs->setVarThresholdGen(bsCombinations.at<double>(i,4));
        pBs->setOpeningSize(bsCombinations.at<double>(i,5));
        
        pBs->model();
        
        pSubtractors[i] = pBs;
    }
    
    // Initialization of visualizer (qualitative) or data structures (quantitative)
    
    cv::FileStorage fs;
    fs.open(path + filename, cv::FileStorage::WRITE);
    fs << "bg-sgmt_combinations" << combinations;
    fs << "scores_indices" << scoresIndices;
    fs << "scores_combinations" << scoresCombinations;
    fs.release();
    
    // Data structures
    scoreds.clear();
    scoreds.resize(combsIndices.size());
    for (int i = 0; i < combsIndices.size(); i++)
    {
        scoreds[i].resize(seqsIndices.size());
        for (int j = 0; j < seqsIndices.size(); j++)
            scoreds[i][j].resize(sequences[seqsIndices[i]]->getMinNumOfFrames());
    }

    // Calculate the errors
    
    for (int i = 0; i < combsIndices.size(); i++)
    {
        int c = combsIndices[i];
        std::cout << "Processing combination " << combinations.row(c) << " .." << endl;
        
        for (int k = 0; k < seqsIndices.size(); k++)
        {
            int s = seqsIndices[k];
            Sequence<ColorDepthFrame>::Ptr pSeq = sequences[s];
            detectionGroundtruths[seqsIndices[k]].setTolerance(combinations.at<double>(i,bsCombinations.cols + 3));

            int f = 0;
            
            // Threading variables
            // ---------------------------------
            boost::thread_group tg;
            // ---------------------------------
            
            boost::timer t;
            
            pSeq->restart();
            while (pSeq->hasNextFrames())
            {
//                if (f < 23)
//                {
//                    pSeq->next();
//                    f++;
//                    continue;
//                }
                // Threading stuff
                // ---------------------------------
                if (tg.size() > 0 && (tg.size() % numOfThreads) == 0)
                {
                    std::cout << "Processing frames [" << (f - numOfThreads) << "," << f << ") in seq " << s << " .. ";
                    t.restart();
                    
                    tg.join_all();
                    
                    std::cout << t.elapsed() << " secs." << std::endl;
                }
                // ---------------------------------
                
                vector<ColorDepthFrame::Ptr> frames = pSeq->nextFrames();
                
                BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS = pSubtractors[bsIndices.at<int>(scoresIndices.at<int>(c,0),0)];
                
                string id = to_str(c) + "-" + to_str(s) + "-" + to_str(f);
                
                std::cout << "Processing frames " << f << " in seq " << s << " .. " << std::endl;
                std::vector<std::vector<pcl::PointXYZ> > detectionPositions;
                _precomputeScores(pSys, frames, pBS, scoresCombinations.row(i), bsCombinations.cols, path + filename, id, scoreds[i][k][f], detectionPositions);
                
                vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, rejections;
                cv::Mat frameErrors;
                detectionGroundtruths[seqsIndices[k]].getFrameSegmentationResults(pSeq->getFrameCounters(), detectionPositions, matches, rejections, frameErrors);
                //visualizeSegmentations(frames, matches, rejections, 0.02, 3);
                visualizeScored(frames, scoreds[i][k][f], 0.02);
                
                // Threading stuff (incl function calling)
                // ---------------------------------------
//                tg.add_thread( new boost::thread(_precomputeScores, pSys, frames, pBS, scoresCombinations.row(i), bsCombinations.cols, path + filename, id, boost::ref(scoreds[i][k][f])) );
                // ---------------------------------------
            
                f++;
            }
            // -----------------------
            if (tg.size() > 0)
            {
                std::cout << "Processing frames [" << (f - (pSeq->getMinNumOfFrames() % numOfThreads)) << "," << f << ") in seq " << s << " .. ";

                tg.join_all();
                
                std::cout << t.elapsed() << " secs." << std::endl;
            }
            // -----------------------
        }
    }
}

float computeAccuracy(std::vector<int> groundtruth, std::vector<int> predictions)
{
    std::map<std::string,int> instancesMap;
    
    for (int i = 0; i < groundtruth.size(); ++i)
        instancesMap[std::to_string((long long) groundtruth[i])]++;
    
    std::map<std::string,int> hitsMap;
    
    std::map<std::string,int>::iterator instancesIt;
    for (instancesIt = instancesMap.begin(); instancesIt != instancesMap.end(); instancesIt++)
        hitsMap[instancesIt->first] = 0;
    
    for (int i = 0; i < groundtruth.size(); ++i)
        if (groundtruth[i] == predictions[i])
            hitsMap[std::to_string((long long) groundtruth[i])]++;
    
    instancesIt = instancesMap.begin();
    std::map<std::string,int>::iterator hitsIt = hitsMap.begin();
    
    float accSum = 0.f;
    while (instancesIt != instancesMap.end())
    {
        accSum += ((float) hitsIt->second) / instancesIt->second;
        instancesIt++;
        hitsIt++;
    }
    
    return accSum / instancesMap.size();
}

float computeAccuracy(cv::Mat groundtruth, cv::Mat predictions)
{
    std::vector<int> _groundtruth (groundtruth.begin<int>(), groundtruth.end<int>());
    std::vector<int> _predictions (predictions.begin<int>(), predictions.end<int>());
    
    return computeAccuracy(_groundtruth, _predictions);
}

void trainObjectRejectionThresholds(std::vector<std::vector<float> > S,
                                    std::vector<int> G, std::vector<double>& bestRjtValues)
{
    bestRjtValues.clear();
    
    // Prepare the matrix of scores
    cv::Mat Sm = wToMat(S);
    cout << Sm << endl; //dbg
    
    // Prepare the binary matrix of groundtruth
    cv::Mat B (Sm.rows, Sm.cols, CV_32S, cv::Scalar(0));
    for (int i = 0; i < G.size(); i++)
        if (G[i] > 0) B.at<int>(i, G[i]-1) = 1; // discar not matched detections
    
    // Prepare the rejection thresholds to be tested
    std::vector<double> rjtValues;
    cvx::linspace(0, 1, 100, rjtValues);
    rjtValues.pop_back(); // value == 1 is a non-sense
    
    // Find the best threshold for each model. As many models as columns of Sm
    bestRjtValues.clear();
    bestRjtValues.resize(Sm.cols);
    std::vector<float> accs (Sm.cols);
    for (int i = 0; i < Sm.cols; i++)
    {
        float maxAcc = 0.f;
        double bestVal;
        for (int t = 0; t < rjtValues.size(); t++)
        {
            cv::Mat p = (Sm.col(i) >= rjtValues[t]) / 255;
            float acc = computeAccuracy(B.col(i), p);
            if (acc > maxAcc)
            {
                bestVal = rjtValues[t];
                maxAcc = acc;
            }
        }
        accs[i] = maxAcc;
        bestRjtValues[i] = bestVal;
    }
}

void a(ReMedi::Ptr pSys, Sequence<ColorDepthFrame>::Ptr pSeq, std::vector<ScoredDetections> scoreds, cv::Mat combination, DetectionOutput dtOut, std::vector<std::vector<float> >& S, std::vector<int>& G)
{
    pSeq->restart();
    int f = 0;
    while (pSeq->hasNextFrames())
    {
        pSeq->next();
        
        std::vector<std::vector< int > > vids;
        std::vector<std::vector< pcl::PointXYZ > > positions;
        std::vector<std::vector< std::vector<float> > > scores;
        scoreds[f].get(vids, positions, scores);
        
        // "OF" from "Output Frame". Is the kind of output DetectionOutput class gets.
        std::vector<std::vector<std::vector<  pcl::PointXYZ  > > > recognitionsOF;
        std::vector<std::vector<std::vector<  std::vector<float>  > > > scoresOF;
        
        void* recognizer = pSys->getObjectRecognizer();
        if (pSys->getDescriptionType() == DESCRIPTION_FPFH)
        {
            ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) recognizer) );
            orc.setRecognitionStrategy(combination.at<double>(0,0));
            orc.setRecognitionConsensus(combination.at<double>(0,1));
            
            orc.setObjectRejection(false);
            orc.recognize(vids, positions, scores, recognitionsOF, scoresOF);
        }
        else if (pSys->getDescriptionType() == DESCRIPTION_PFHRGB)
        {
            ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) recognizer) );
            orc.setRecognitionStrategy(combination.at<double>(0,0));
            orc.setRecognitionConsensus(combination.at<double>(0,1));
            
            orc.setObjectRejection(false);
            orc.recognize(vids, positions, scores, recognitionsOF, scoresOF);
        }
        
//        vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, rejections;
//        cv::Mat frameErrors;
//        dtOut.getFrameRecognitionResults(pSeq->getFrameCounters(), recognitionsOF, matches, rejections, frameErrors); // TODO: get from somewhere the frame counters frame ids
//
//        vector<ColorDepthFrame::Ptr> frames = pSeq->getFrames(f);
//        visualizeRecognitions(frames, matches, rejections, 0.02, 3);
        
        std::vector<std::vector<std::vector<  int  > > > groundtruthOF;
        dtOut.getRecognitionGroundtruth(pSeq->getFrameCounters(), recognitionsOF, groundtruthOF);
        
        assert( scoresOF.size() == groundtruthOF.size() );
        for (int v = 0; v < scoresOF.size(); v++)
        {
            assert( scoresOF[v].size() == groundtruthOF[v].size() );
            for (int o = 0; o < scoresOF[v].size(); o++)
            {
                assert( scoresOF[v][o].size() == groundtruthOF[v][o].size() );
                for (int i = 0; i < scoresOF[v][o].size(); i++)
                {
                    S.push_back(scoresOF[v][o][i]);
                    G.push_back(groundtruthOF[v][o][i]);
                }
            }
        }
        
        f++;
    }
}


void computeRejectionThresholds(ReMedi::Ptr pSys, std::vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsSubjectIds, std::vector<int> seqsIndices, cv::Mat sgmtCombinations, cv::Mat rcgnCombinations, cv::Mat combinations, std::vector<std::vector<std::vector<ScoredDetections> > > scoreds, std::vector<DetectionOutput> detectionGroundtruths, std::vector<std::vector<std::vector<double> > >& rejections)
{
    rejections.clear();
     // #{combinations} x #{subjects} x #{models}
    rejections.resize(combinations.rows);
    for (int i = 0; i < combinations.rows; i++)
        rejections[i].resize(NUM_OF_SUBJECTS);
    
    for (int c = 0; c < combinations.rows; c++)
    {
        for (int j = 0; j < NUM_OF_SUBJECTS; j++)
        {
            std::vector<std::vector<float> > S; // auxiliary to "rejections"
            std::vector<int> G; // auxiliary to "rejections"

            for (int s = 0; s < seqsIndices.size(); s++)
            {
                int sid = seqsIndices[s]; // sequence real identifier (0to30) indexed by seqIndices[s]
                if (j == seqsSubjectIds[sid]) continue;
                
                Sequence<ColorDepthFrame>::Ptr pSeq = sequences[sid];
                detectionGroundtruths[sid].setTolerance(combinations.at<double>(c, sgmtCombinations.cols-1)); // TODO: check detec gt var index, instead of 9
                
                cv::Mat rcgnCombination = combinations.row(c).colRange(sgmtCombinations.cols, combinations.cols);
                a(pSys, pSeq, scoreds[c/rcgnCombinations.rows][s], rcgnCombination, detectionGroundtruths[sid], S, G);
            }
            
            trainObjectRejectionThresholds(S, G, rejections[c][j]);
        }
    }
}

void validateMonitorizationRecognition(ReMedi::Ptr pSys, std::vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsSubjectIds, std::vector<int> seqsIndices, cv::Mat sgmtCombinations, std::vector<std::vector<std::vector<ScoredDetections> > > scoreds, std::vector<std::vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<cv::Mat> >& errors, bool bQualitativeEvaluation)
{
    cv::Mat rcgnCombinations;
    expandParameters<double>(rcgnParameters, rcgnCombinations);
    
    cv::Mat combinations; // #{sgmt combs} * #{rcgn combs}
    cvx::combine(sgmtCombinations, rcgnCombinations, 1, combinations);
    
//    std::vector<std::vector<std::vector<double> > > rejections;
//    computeRejectionThresholds(pSys, sequences, seqsSubjectIds, seqsIndices, sgmtCombinations, rcgnCombinations, combinations, scoreds, detectionGroundtruths, rejections);
    
//    if (!bQualitativeEvaluation)
//    {
//        errors.clear();
//        errors.resize(combinations.rows);
//        for (int c = 0; c < combinations.rows; c++)
//        {
//            errors[c].resize(seqsIndices.size()); // sequences (indexed)
//            for (int j = 0; j < seqsIndices.size(); j++)
//            {
//                errors[c][j] = cv::Mat(sequences[seqsIndices[j]]->getNumOfViews(),
//                                          sequences[seqsIndices[j]]->getMinNumOfFrames(),
//                                          CV_32SC3, cv::Scalar(0));
//            }
//        }
//    }
    
    std::vector<std::vector<std::vector<ScoredDetections> > > scoredsAux = scoreds; // debug
    
    for (int c = 0; c < combinations.rows; c++)
    {
        for (int s = 0; s < seqsIndices.size(); s++)
        {
            int sid = seqsIndices[s];
            
            Sequence<ColorDepthFrame>::Ptr pSeq = sequences[sid];
            detectionGroundtruths[sid].setTolerance(combinations.at<double>(c, sgmtCombinations.cols-1)); // TODO: check detec gt var index, instead of 9
            
            pSeq->restart();
            int f = 0;
            while (pSeq->hasNextFrames())
            {
                pSeq->next();
                
                std::vector<std::vector< int > > vids;
                std::vector<std::vector< pcl::PointXYZ > > positions;
                std::vector<std::vector< std::vector<float> > > scores;

                scoreds[c/rcgnCombinations.rows][s][f].get(vids, positions, scores); // BAD ACCESS ERROR here because of f > 2. There are not scores precomputed yet!

                // "OF" from "Output Frame". Is the kind of output DetectionOutput class gets.
                std::vector<std::vector<std::vector<  pcl::PointXYZ  > > > recognitionsOF;
                std::vector<std::vector<std::vector<  std::vector<float>  > > > scoresOF;
                
                void* recognizer = pSys->getObjectRecognizer();
                if (pSys->getDescriptionType() == DESCRIPTION_FPFH)
                {
                    ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) recognizer) );
                    orc.setRecognitionStrategy(combinations.at<double>(c, sgmtCombinations.cols + 0));
                    orc.setRecognitionConsensus(combinations.at<double>(c, sgmtCombinations.cols + 1));
                    
                    orc.setObjectRejection(false);
                    orc.recognize(vids, positions, scores, recognitionsOF, scoresOF);
                }
                else if (pSys->getDescriptionType() == DESCRIPTION_PFHRGB)
                {
                    ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) recognizer) );
                    orc.setRecognitionStrategy(combinations.at<double>(c, sgmtCombinations.cols + 0));
                    orc.setRecognitionConsensus(combinations.at<double>(c, sgmtCombinations.cols + 1));
                    
                    orc.setObjectRejection(false);
                    orc.recognize(vids, positions, scores, recognitionsOF, scoresOF);
                }
                
                vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, rejections;
                cv::Mat frameErrors;
                detectionGroundtruths[sid].getFrameRecognitionResults(pSeq->getFrameCounters(), recognitionsOF, matches, rejections, frameErrors); // TODO: get from somewhere the frame counters frame ids

                if (bQualitativeEvaluation)
                {
                  vector<ColorDepthFrame::Ptr> frames = pSeq->getFrames(f);
                  visualizeRecognitions(frames, matches, rejections, 0.02, 3);
                }
                
                f++;
            }
        }
    }
//
//    if (!bQualitativeEvaluation)
//    {
//        cv::FileStorage fs;
//        fs.open(path + filename, cv::FileStorage::WRITE);
//        fs << "seqs_indices" << seqsIndices;
//        fs << "sgmt_combinations" << sgmtCombinations;
//        fs << "rcgn_combinations" << rcgnCombinations;
//        fs << "combinations" << combinations;
//        
//        for (int i = 0; i < combinations.rows; i++) for (int k = 0; k < seqsIndices.size(); k++)
//        {
//            string id = "combination_" + to_str(i) + "-" + to_str(seqsIndices[k]);
//            // errors[i][k] is a #{NUM_OF_VIEWS} x #{number of frames in sequence 'seqsIndices[k]'}
//            // mat with 3-channels, representing TP,FN,FP.
//            fs << id << errors[i][k];
//        }
//        fs.release();
//    }
}

// Faster version
void validateMonitorizationRecognition(vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bgSgmtCombinations, vector<vector<vector<cv::Mat> > > positions, vector<vector<vector<cv::Mat> > > scores, vector<vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation)
{
    cv::Mat rcgnCombinations;
    expandParameters<double>(rcgnParameters, rcgnCombinations);
    
    
    for (int i = 0; i < rcgnCombinations.rows; i++)
    {
        for (int c = 0; c < scores.size(); c++)
        {
            for (int s = 0; s < scores[c].size(); s++)
            {
                Sequence<ColorDepthFrame>::Ptr pSeq = sequences[s];
                
                vector<vector<vector<pcl::PointXYZ> > > recognitions; // previous recognitions
                detectionGroundtruths[s].setTolerance(bgSgmtCombinations.at<double>(c, 9)); // TODO: check detec gt var index, instead of 9

                int f = 0;
    
                pSeq->restart();
                while (pSeq->hasNextFrames())
                {
                    pSeq->next();

                    // TODO: build "recognitions" (^) from positions and scores
                    // Access: with positions[c][s][f] and scores[c][s][f]
                    
                    vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, rejections;
                    cv::Mat frameErrors;
                    detectionGroundtruths[s].getFrameRecognitionResults(pSeq->getFrameCounters(), recognitions, matches, rejections, frameErrors); // TODO: get from somewhere the frame counters frame ids
                    
                    if (bQualitativeEvaluation)
                    {
                        vector<ColorDepthFrame::Ptr> frames = pSeq->getFrames(f);
                        visualizeRecognitions(frames, matches, rejections, 0.02, 3);
                    }
                    else
                    {

                        // TODO: keep record of the errors
                        // Modify this line: frameErrors.copyTo(errors[i][j][s].col(f));
                    }
                    
                    f++;
                }
            }
        }
    }
//    
//    
//        
//    
//    
//        
//
//        
//    }
//    
//            // DEBUG
//            // <-----------------------------------------------------------------------------------------------
//            for (int i = 0; i < rcgnCombinations.rows; i++) for (int j = 0; j < combinations.rows; j++)
//            {
//                cv::Mat errorsF;
//                errors[i][j][s].convertTo(errorsF, CV_32FC(errors[i][j][s].channels()));
//                cv::Mat errorsRoi (errorsF, cv::Rect(0, 0, f+1, errorsF.rows));
//                
//                cv::Mat errorsRoiReduced;
//                cv::reduce(errorsRoi, errorsRoiReduced, 1, CV_REDUCE_SUM);
//                
//                cout << (i * combinations.rows + j) << " " << i << " " << j << " : ";
//                for (int v = 0; v < errorsRoiReduced.rows; v++)
//                {
//                    cv::Vec3f err = errorsRoiReduced.at<cv::Vec3f>(v,0);
//                    float fscore = computeF1Score(err.val[0], err.val[1], err.val[2]);
//                    cout << (v > 0 ? ", " : "") << to_string_with_precision<float>(fscore);
//                } cout << endl;
//            }
//            // ----------------------------------------------------------------------------------------------->
//            
//            f++;
//            cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
//        }
//    }
//    
//    if (!bQualitativeEvaluation)
//    {
//        cv::FileStorage fs;
//        fs.open(path + filename, cv::FileStorage::APPEND);
//        for (int i = 0; i < rcgnCombinations.rows; i++)
//        {
//            for (int j = 0; j < combinations.rows; j++)
//            {
//                for (int s = 0; s < sequences.size(); s++)
//                {
//                    string id = "combination_" + to_str(i) + "-" + to_str(j) + "-" + to_str(s);
//                    fs << id << errors[i][j][s];
//                }
//            }
//        }
//        fs.release();
//    }
}

//// Faster version
//void validateMonitorizationRecognition(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat combinations, vector<vector<double> > rcgnParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation)
//{
//    pSys->initialize();
//    
//    Sequence<ColorDepthFrame>::Ptr pBgSeq = pSys->getBackgroundSequence();
//    
//    // Some consistency checks
//    // -------------------------------------------------
//    int numOfViews = pBgSeq->getNumOfViews();
//    for (int s = 0; s < sequences.size(); s++)
//        assert (numOfViews == sequences[s]->getNumOfViews());
//    // -------------------------------------------------
//    
//    cv::Mat rcgnCombinations;
//    expandParameters<double>(rcgnParameters, rcgnCombinations);
//    
//    // Do not repeat the bs every time. Pre-compute the required combinations!
//    cv::Mat bsCombinations, bsIndices;
//    cvx::unique(combinations.colRange(0,6), 0, bsCombinations, bsIndices); // separate (bs,sgmn) parameters in combinations and find unique bs' sub-combinations
//    
//    vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors (bsCombinations.rows);
//    
//    for (int i = 0; i < bsCombinations.rows; i++)
//    {
//        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs (new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
//        *pBs = *(pSys->getBackgroundSubtractor());
//        
//        pBs->setModality(bsCombinations.at<double>(i,0));
//        pBs->setNumOfMixtureComponents(bsCombinations.at<double>(i,1));
//        pBs->setLearningRate(bsCombinations.at<double>(i,2));
//        pBs->setBackgroundRatio(bsCombinations.at<double>(i,3));
//        pBs->setVarThresholdGen(bsCombinations.at<double>(i,4));
//        pBs->setOpeningSize(bsCombinations.at<double>(i,5));
//        
//        pBs->model();
//        
//        pSubtractors[i] = pBs;
//    }
//    
//    // Same for object recognizer (and its internal cloudjectmodels)
//    
//    cv::Mat descCombinations, descIndices;
//    cvx::unique(rcgnCombinations.colRange(0,1), 0, descCombinations, descIndices); // separate (descs,strategies) parameters in combinations and find unique bs' sub-combinations
//    
//    vector<void*> recognizers (descCombinations.rows);
//    
//    for (int i = 0; i < descCombinations.rows; i++)
//    {
//        int dType = (int) descCombinations.at<double>(i,0);
//        if (dType == DESCRIPTION_FPFH)
//        {
//            if (pSys->getDescriptionType() == dType)
//                recognizers[i] = pSys->getObjectRecognizer();
//            else
//            {
//                ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>* pOR = new ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>();
//                
//                if (pSys->getDescriptionType() == DESCRIPTION_PFHRGB)
//                    *pOR = *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) pSys->getObjectRecognizer());
//                // else if (in case of more kinds of descriptions)
//                // ...
//                
//                pOR->create();
//                
//                recognizers[i] = (void*) pOR;
//            }
//        }
//        else if (dType == DESCRIPTION_PFHRGB)
//        {
//            if (pSys->getDescriptionType() == dType)
//                recognizers[i] = pSys->getObjectRecognizer();
//            else
//            {
//                ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>* pOR = new ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>();
//                
//                if (pSys->getDescriptionType() == DESCRIPTION_FPFH)
//                    *pOR = *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) pSys->getObjectRecognizer());
//                // else if (in case of more kinds of descriptions)
//                // ...
//                
//                pOR->create();
//                
//                recognizers[i] = (void*) pOR;
//            }
//        }
//    }
//    
//    // Initialization of visualizer (qualitative) or data structures (quantitative)
//    
//    if (!bQualitativeEvaluation)
//    {
//        cv::FileStorage fs;
//        fs.open(path + filename, cv::FileStorage::WRITE);
//        fs << "rcgn_combinations" << rcgnCombinations;
//        fs << "bs-sgmn_combinations" << combinations;
//        fs.release();
//        
//        // Data structures
//        
//        errors.resize(rcgnCombinations.rows);
//        for (int i = 0; i < errors.size(); i++)
//        {
//            errors[i].resize(combinations.rows);
//            for (int j = 0; j < errors[i].size(); j++)
//            {
//                errors[i][j].resize(sequences.size());
//                for (int s = 0; s < errors[i][j].size(); s++)
//                {
//                    errors[i][j][s] = cv::Mat(sequences[s]->getNumOfViews(),
//                                              sequences[s]->getMinNumOfFrames(),
//                                              CV_32SC3, cv::Scalar(0));
//                }
//            }
//        }
//        
//    }
//    
//    // Calculate the errors
//    
//    for (int s = 0; s < sequences.size(); s++)
//    {
//        Sequence<ColorDepthFrame>::Ptr pSeq = sequences[s];
//        
//        int f = 0;
//        
//        pSeq->restart();
//        while (pSeq->hasNextFrames())
//        {
//            boost::timer t;
//            vector<ColorDepthFrame::Ptr> frames = pSeq->nextFrames();
//            
//            pSys->getRegisterer()->setInputFrames(frames);
//            pSys->getRegisterer()->registrate(frames);
//            
//            for (int i = 0; i < rcgnCombinations.rows; i++)
//            {
//                vector<vector< pair<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > > detectionCorrespondences;
//                vector<vector<vector<pcl::PointXYZ> > > recognitions; // previous recognitions
//                
//                for (int j = 0; j < combinations.rows; j++)
//                {
//                    vector<cv::Mat> foregroundMasks;
//                    pSubtractors[bsIndices.at<int>(j,0)]->setInputFrames(frames);
//                    pSubtractors[bsIndices.at<int>(j,0)]->subtract(foregroundMasks);
//                    
//                    vector<cv::Mat> tabletopMasks;
//                    pSys->getTableModeler()->getTabletopMask(frames, tabletopMasks);
//                    
//                    for (int v = 0; v < pSeq->getNumOfViews(); v++)
//                    {
//                        frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
//                    }
//                    
//                    ObjectDetector::Ptr pObjectDetector = pSys->getObjectDetector();
//                    pObjectDetector->setInputFrames(frames);
//                    
//                    pObjectDetector->setDownsamplingSize(combinations.at<double>(j, bsCombinations.cols + 0));
//                    pObjectDetector->setClusteringIntradistanceFactor(combinations.at<double>(j, bsCombinations.cols + 1));
//                    pObjectDetector->setMinClusterSize(combinations.at<double>(j, bsCombinations.cols + 2));
//                    pObjectDetector->setInterviewCorrepondenceDistance(combinations.at<double>(j, bsCombinations.cols + 3));
//                    
//                    pObjectDetector->detect();
//                    if ((int) rcgnCombinations.at<double>(i,1) == RECOGNITION_MONOCULAR)
//                        pObjectDetector->getDetectionCorrespondences(detectionCorrespondences, false);
//                    else
//                        pObjectDetector->getDetectionCorrespondences(detectionCorrespondences, true);
//                    
//                    int dType = (int) rcgnCombinations.at<double>(i,0);
//                    if (dType == DESCRIPTION_FPFH)
//                    {
//                        ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>* pOR = (ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) recognizers[descIndices.at<int>(i,0)];
//                        pOR->setInputDetections(detectionCorrespondences);
//                        pOR->recognize(recognitions);
//                    }
//                    else if (dType == DESCRIPTION_PFHRGB)
//                    {
//                        ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>* pOR = (ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) recognizers[descIndices.at<int>(i,0)];
//                        pOR->setInputDetections(detectionCorrespondences);
//                        pOR->recognize(recognitions);
//                    }
//                    
//                    detectionGroundtruths[s].setTolerance(combinations.at<double>(j, bsCombinations.cols + 3));
//                    
//                    vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, rejections;
//                    cv::Mat frameErrors;
//                    detectionGroundtruths[s].getFrameRecognitionResults(pSeq->getFrameCounters(), recognitions, matches, rejections, frameErrors);
//                    
//                    if (bQualitativeEvaluation)
//                        visualizeRecognitions(frames, matches, rejections, 0.02, 3);
//                    else
//                        frameErrors.copyTo(errors[i][j][s].col(f));
//                }
//            }
//            
//            // DEBUG
//            // <-----------------------------------------------------------------------------------------------
//            for (int i = 0; i < rcgnCombinations.rows; i++) for (int j = 0; j < combinations.rows; j++)
//            {
//                cv::Mat errorsF;
//                errors[i][j][s].convertTo(errorsF, CV_32FC(errors[i][j][s].channels()));
//                cv::Mat errorsRoi (errorsF, cv::Rect(0, 0, f+1, errorsF.rows));
//                
//                cv::Mat errorsRoiReduced;
//                cv::reduce(errorsRoi, errorsRoiReduced, 1, CV_REDUCE_SUM);
//                
//                cout << (i * combinations.rows + j) << " " << i << " " << j << " : ";
//                for (int v = 0; v < errorsRoiReduced.rows; v++)
//                {
//                    cv::Vec3f err = errorsRoiReduced.at<cv::Vec3f>(v,0);
//                    float fscore = computeF1Score(err.val[0], err.val[1], err.val[2]);
//                    cout << (v > 0 ? ", " : "") << to_string_with_precision<float>(fscore);
//                } cout << endl;
//            }
//            // ----------------------------------------------------------------------------------------------->
//            
//            f++;
//            cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
//        }
//    }
//    
//    if (!bQualitativeEvaluation)
//    {
//        cv::FileStorage fs;
//        fs.open(path + filename, cv::FileStorage::APPEND);
//        for (int i = 0; i < rcgnCombinations.rows; i++)
//        {
//            for (int j = 0; j < combinations.rows; j++)
//            {
//                for (int s = 0; s < sequences.size(); s++)
//                {
//                    string id = "combination_" + to_str(i) + "-" + to_str(j) + "-" + to_str(s);
//                    fs << id << errors[i][j][s];
//                }
//            }
//        }
//        fs.release();
//    }
//}

void visualizeScored(vector<ColorDepthFrame::Ptr> frames, ScoredDetections scored, float markersRadius)
{
    cv::Mat positionsSparse, scoresSparse;
    scored.getSparseRepresentation(positionsSparse, scoresSparse);
    
    // Create visualizer
    pcl::visualization::PCLVisualizer::Ptr pVis ( new pcl::visualization::PCLVisualizer );
    
    // Create viewports (horizontally)
    vector<int> viewports (positionsSparse.cols);
    for (int v = 0; v < positionsSparse.cols; v++)
    {
        pVis->createViewPort(v * (1.f/viewports.size()), 0, (v+1) * (1.f/viewports.size()), 1, viewports[v]);
    }
    
    // Draw clouds, matches, and rejections
    
    for (int v = 0; v < positionsSparse.cols; v++)
    {
        // Annotations are not registered, so get the unregistered point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColoredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        frames[v]->getColoredPointCloud( *pColoredCloud ); // unregistered
        
        pVis->addPointCloud(pColoredCloud, "cloud_" + to_str(v), viewports[v]);
        
        for (int i = 0; i < positionsSparse.rows; i++)
        {
            cv::Vec3f p = positionsSparse.at<cv::Vec3f>(i,v);
            if (!(p[0] == 0 && p[1] == 0 && p[2] == 0))
                pVis->addSphere(pcl::PointXYZ(p[0],p[1],p[2]), markersRadius, g_Colors[i][2], g_Colors[i][1], g_Colors[i][0], "detection_" + to_str(v) + to_str(i), viewports[v]);
        }
    }
    
    pVis->spin();
}

void visualizeRecognitions(vector<ColorDepthFrame::Ptr> frames, vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > rejections, float markersRadius, float lineWidth)
{
    // Create visualizer
    pcl::visualization::PCLVisualizer::Ptr pVis ( new pcl::visualization::PCLVisualizer );
    
    // Create viewports (horizontally)
    vector<int> viewports (frames.size());
    for (int v = 0; v < viewports.size(); v++)
    {
        pVis->createViewPort(v * (1.f/viewports.size()), 0, (v+1) * (1.f/viewports.size()), 1, viewports[v]);
        //pVis->addCoordinateSystem(0.1, 0, 0, 0, "cs" + to_str(v), viewports[v]);
    }
    
    // Draw clouds, matches, and rejections
    
    for (int v = 0; v < frames.size(); v++)
    {
        // Annotations are not registered, so get the unregistered point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColoredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        frames[v]->getColoredPointCloud( *pColoredCloud ); // unregistered
        
        pVis->addPointCloud(pColoredCloud, "cloud_" + to_str(v), viewports[v]);
        
        pcl::PointXYZ p, q;
        
        // Groundtruth annotations represented as yellow spheres
        // matches are in green linked with green line to groundtruth
        vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > objects = matches[v];
        for (int o = 0; o < matches[v].size(); o++) for (int i = 0; i < matches[v][o].size(); i++)
        {
            p = matches[v][o][i].first;
            q = matches[v][o][i].second;
            
            pVis->addSphere(p, markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "matches_prediction_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "matches_groundtruth_"  + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_groundtruth_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            
            pVis->addLine(p, q, 0, 1, 0, "matches_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
        }
        // Rejections are in red linked with red line to groundtruth
        for (int o = 0; o < rejections[v].size(); o++)  for (int i = 0; i < rejections[v][o].size(); i++)
        {
            p = rejections[v][o][i].first;
            q = rejections[v][o][i].second;
            
            if (!(p.x == 0 && p.y == 0 && p.z == 0))
                pVis->addSphere(p, markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_prediction_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            
            if (!(q.x == 0 && q.y == 0 && q.z == 0))
            {
                pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_groundtruth_"  + to_str(v) + to_str(o) + to_str(i), viewports[v]);
                pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_groundtruth_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            }
            
            if (!(p.x == 0 && p.y == 0 && p.z == 0) && !(q.x == 0 && q.y == 0 && q.z == 0))
            {
                pVis->addLine(p, q, 1, 0, 0, "rejections_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
                pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            }
        }
    }
    
    pVis->spin();
}