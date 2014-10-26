 //
//  recognition_validation.cpp
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#include "recognition_validation.h"
#include <boost/thread.hpp>

boost::mutex g_Mutex;

void loadPrecomputedRecognitionScoresFile(std::string filePath, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& positions, vector<vector<vector<cv::Mat> > >& scores)
{
    combinations.release();
    positions.clear();
    scores.clear();
    
    cv::FileStorage fs;
    fs.open(filePath, cv::FileStorage::READ);
    
    fs["bs-sgmn_combinations"] >> combinations;
    
    positions.resize(combinations.rows);
    scores.resize(combinations.rows);
    for (int i = 0; i < combinations.rows; i++)
    {
        std::vector<std::vector<cv::Mat> > positionsComb, scoresComb;
        std::vector<cv::Mat> positionsSeq, scoresSeq;
        int s = 0;
        do
        {
            cv::Mat positionsFrame, scoresFrame;
            int f = 0;
            do
            {
                string id = to_str(i) + "-" + to_str(s) + "-" + to_str(f);
                fs["positions_" + id] >> positionsFrame;
                fs["scores_" + id] >> scoresFrame;
                
                if (!scoresFrame.empty())
                {
                    positionsSeq.push_back(positionsFrame);
                    scoresSeq.push_back(scoresFrame);
                }
                f++;
            }
            while (!scoresFrame.empty());
            
            if (!scoresSeq.empty())
            {
                positionsComb.push_back(positionsSeq);
                scoresComb.push_back(scoresSeq);
            }
            s++;
        }
        while (!scoresSeq.empty());
        
        positions[i] = positionsComb;
        scores[i] = scoresComb;
    }
    
    fs.release();
}

void ScoredDetections::toSparseRepresentation(cv::Mat& positions, cv::Mat& scores)
{
    positions.create(positions_.size(), NUM_OF_VIEWS, CV_32FC(3));
    scores.create(scores_.size(), NUM_OF_VIEWS * OD_NUM_OF_OBJECTS, CV_32FC1);
    
    positions.setTo(0);
    scores.setTo(0);
    
    for (int i = 0; i < vids_.size(); i++)
    {
        for (int j = 0; j < vids_[i].size(); j++)
        {
            int vid = vids_[i][j];
            
            pcl::PointXYZ p = positions_[i][j];
            positions.at<cv::Vec3f>(i,vid) = cv::Vec3f(p.x, p.y, p.z);
            
            for (int k = 0; k < OD_NUM_OF_OBJECTS; k++)
                scores.at<float>(i, vid * OD_NUM_OF_OBJECTS + k) = scores_[i][j][k];
        }
    }
}

void _precomputeScores(ReMedi::Ptr pSys, vector<ColorDepthFrame::Ptr> frames, BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS, cv::Mat combination, int offset, std::string filePath, std::string id, ScoredDetections& scoreds)
{
    pSys->getRegisterer()->setInputFrames(frames);
    pSys->getRegisterer()->registrate(frames);
    
    vector<cv::Mat> foregroundMasks;
    pBS->setInputFrames(frames);
    pBS->subtract(foregroundMasks);
    
    vector<cv::Mat> tabletopMasks;
    pSys->getTableModeler()->getTabletopMask(frames, tabletopMasks);
    
    for (int v = 0; v < frames.size(); v++)
        frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
    
    ObjectDetector od (*pSys->getObjectDetector());
    od.setInputFrames(frames);
    
    od.setDownsamplingSize(combination.at<double>(0, offset + 0));
    od.setClusteringIntradistanceFactor(combination.at<double>(0, offset + 1));
    od.setMinClusterSize(combination.at<double>(0, offset + 2));
//    od.setInterviewCorrepondenceDistance(combination.at<double>(0, offset + 3));
    
    od.detect();
    
    vector<vector< pair<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > > detectionCorrespondences;
    od.getDetectionCorrespondences(detectionCorrespondences, false);
    
    void* recognizer = pSys->getObjectRecognizer();
    if (pSys->getDescriptionType() == DESCRIPTION_FPFH)
    {
        ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) recognizer) );
        orc.setInputDetections(detectionCorrespondences);
        orc.getScores(scoreds.vids_, scoreds.positions_, scoreds.scores_);
    }
    else if (pSys->getDescriptionType() == DESCRIPTION_PFHRGB)
    {
        ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250> orc ( *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) recognizer) );
        orc.setInputDetections(detectionCorrespondences);
        orc.getScores(scoreds.vids_, scoreds.positions_, scoreds.scores_);
    }
    
    // To proper format and save
    
    cv::Mat positions, scores;
    scoreds.toSparseRepresentation(positions, scores);
    
    g_Mutex.lock();
    // ---
    cv::FileStorage fs;
    fs.open(filePath, cv::FileStorage::APPEND);
    if (fs.isOpened())
    {
        fs << ("positions_" + id) << positions;
        fs << ("scores_" + id) << scores;
        fs.release();
    }
    // ---
    g_Mutex.unlock();
}

void precomputeRecognitionScores(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsIndices, cv::Mat combinations, std::vector<int> combsIndices, string path, string filename, vector<vector<vector<ScoredDetections> > >& scoreds)
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
    cvx::unique(combinations.colRange(0,combinations.cols-1), 0, scoresCombinations, scoresIndices);
    
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
            
            int f = 0;
            
            // Threading variables
            // ---------------------------------
            boost::thread_group tg;
            std::vector<boost::thread*> actives;
            // ---------------------------------
            
            boost::timer t;
            
            pSeq->restart();
            while (pSeq->hasNextFrames())
            {
                // Threading stuff
                // ---------------------------------
                if (tg.size() > 0 && (tg.size() % NUM_OF_THREADS) == 0)
                {
                    std::cout << "Processing frames [" << (f - NUM_OF_THREADS) << "," << f << "] in seq " << s << " .. ";
                    t.restart();
                    
                    tg.join_all();

                    for (int t = 0; t < actives.size(); t++)
                        tg.remove_thread(actives[t]);
                    
                    actives.clear();
                    
                    cout << t.elapsed() << " secs." << endl;
                }
                // ---------------------------------
                
                vector<ColorDepthFrame::Ptr> frames = pSeq->nextFrames();

                BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS = pSubtractors[bsIndices.at<int>(scoresIndices.at<int>(c,0),0)];
                
                string id = to_str(c) + "-" + to_str(s) + "-" + to_str(f);
                
//                _precomputeScores(pSys, frames, pBS, scoresCombinations.row(i), bsCombinations.cols, path + filename, id, scoreds[i][k][f]);
                
                // Threading stuff (incl function calling)
                // ---------------------------------------
                boost::thread* pThread = new boost::thread( _precomputeScores, pSys, frames, pBS, scoresCombinations.row(i), bsCombinations.cols, path + filename, id, boost::ref(scoreds[i][k][f]) );
                tg.add_thread(pThread);
                actives.push_back(pThread);
                // ---------------------------------------
            
                f++;
            }
            // -----------------------
            tg.join_all();
            // -----------------------
        }
    }
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
            
            pVis->addSphere(p, markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_prediction_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_groundtruth_"  + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_groundtruth_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            
            pVis->addLine(p, q, 1, 0, 0, "rejections_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_line_" + to_str(v) + to_str(o) + to_str(i), viewports[v]);
        }
    }
    
    pVis->spin();
}