//
//  segmentation_validation.cpp
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#include "segmentation_validation.h"

//void loadMonitorizationSegmentationValidationFile(string filePath, cv::Mat& bsCombinations, cv::Mat& mntrCombinations, vector<vector<vector<cv::Mat> > >& errors)
//{
//    cv::FileStorage fs;
//    fs.open(filePath, cv::FileStorage::READ);
//    
//    fs["bs_combinations"] >> bsCombinations;
//    fs["mntr_combinations"] >> mntrCombinations;
//    
//    errors.resize(bsCombinations.rows);
//    for (int i = 0; i < bsCombinations.rows; i++)
//    {
//        errors[i].resize(mntrCombinations.rows);
//        for (int j = 0; j < mntrCombinations.rows; j++)
//        {
//            cv::Mat m;
//            
//            int s = 0;
//            do
//            {
//                string id = "combination_" + to_str(i) + "-" + to_str(j) + "-" + to_str(s++);
//                fs[id] >> m;
//                
//                if (!m.empty()) errors[i][j].push_back(m);
//            }
//            while (!m.empty());
//            
//        }
//    }
//    fs.release();
//    
//    fs.open(filePath + "2", cv::FileStorage::WRITE);
//    
//    cv::Mat combinations;
//    cvx::combine(bsCombinations, mntrCombinations, 1, combinations);
//    fs << "combinations" << combinations;
//    
//    for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinations.rows; j++)
//    {
//        for (int s = 0; s < errors[i][j].size(); s++)
//        {
//            string id = "combination_" + to_str(i * mntrCombinations.rows + j) + "-" + to_str(s);
//            fs << id << errors[i][j][s];
//        }
//    }
//    
//    fs.release();
//}

void loadMonitorizationSegmentationValidationFile2(string filePath, cv::Mat& combinations, vector<vector<cv::Mat> >& errors)
{
    cv::FileStorage fs;
    fs.open(filePath, cv::FileStorage::READ);
    fs["combinations"] >> combinations;
    
    errors.resize(combinations.rows);
    for (int i = 0; i < combinations.rows; i++)
    {
        cv::Mat m;
        
        int s = 0;
        do
        {
            string str = "combination_" + to_str(i) + "-" + to_str(s++);
            fs[str.c_str()] >> m;
            
            if (!m.empty()) errors[i].push_back(m);
        }
        while (!m.empty());
    }
    
    fs.release();
}

//// Faster version
//void validateMonitorizationSegmentation(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation)
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
//    vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors;
//    pSubtractors.resize(bsCombinations.rows);
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
//    cv::Mat mntrCombinations;
//    expandParameters<double>(mntrParameters, mntrCombinations);
//
//    ObjectDetector::Ptr pObjectDetector = pSys->getObjectDetector();
//
//    // Initialization of visualizer (qualitative) or data structures (quantitative)
//
//    int offset = 0; // in case quantitatively evaluate and load previous results from file
//
//    if (!bQualitativeEvaluation)
//    {
//
//        cv::FileStorage fs;
//
//        // Backup file
//
//        boost::filesystem::path filePath (path + filename);
//        if (!boost::filesystem::exists(filePath))
//        {
//            fs.open(path + filename, cv::FileStorage::WRITE);
//            fs << "bs_combinations" << bsCombinations;
//            fs << "mntr_combinations" << mntrCombinations;
//            fs.release();
//
//            // Data structures
//
//            errors.resize(bsCombinations.rows);
//            for (int i = 0; i < bsCombinations.rows; i++)
//            {
//                errors[i].resize(mntrCombinations.rows);
//                for (int j = 0; j < mntrCombinations.rows; j++)
//                {
//                    errors[i][j].resize(sequences.size());
//                    for (int s = 0; s < sequences.size(); s++)
//                    {
//                        errors[i][j][s].create(sequences[s]->getNumOfViews(), sequences[s]->getMinNumOfFrames(), CV_32SC3);
//                    }
//                }
//            }
//        }
//        else
//        {
//            boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
//            boost::filesystem::copy_file(filePath, fileBakPath, boost::filesystem::copy_option::overwrite_if_exists);
//
//            cv::Mat bsCombinationsTmp, mntrCombinationsTmp;
//            vector<vector<vector<cv::Mat> > > errorsTmp;
//            loadMonitorizationSegmentationValidationFile(path + filename, bsCombinationsTmp, mntrCombinationsTmp, errorsTmp);
//
//            cv::Mat mntrMatchedCombinations, mntrPendingCombinations;
//            cvx::match(mntrCombinationsTmp, mntrCombinations, mntrMatchedCombinations, mntrPendingCombinations);
//
//            fs.open(path + filename, cv::FileStorage::WRITE);
//
//            fs << "bs_combinations" << bsCombinations;
//
//            if (mntrPendingCombinations.empty())
//            {
//                fs << "mntr_combinations" << mntrCombinationsTmp;
//            }
//            else
//            {
//                cv::Mat aux;
//                cv::vconcat(mntrCombinationsTmp, mntrPendingCombinations, aux);
//
//                fs << "mntr_combinations" << aux;
//            }
//
//            // Data structures
//            offset = mntrCombinationsTmp.rows;
//            int n = offset + mntrPendingCombinations.rows;
//
//            errors.resize(bsCombinations.rows);
//            for (int i = 0; i < bsCombinations.rows; i++)
//            {
//                errors[i].resize(n);
//                for (int j = 0; j < n; j++)
//                {
//                    errors[i][j].resize(sequences.size());
//                    for (int s = 0; s < sequences.size(); s++)
//                    {
//                        errors[i][j][s].create(sequences[s]->getNumOfViews(), sequences[s]->getMinNumOfFrames(), CV_32SC3);
//                    }
//                }
//            }
//
//            for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinationsTmp.rows; j++)
//            {
//                for (int s = 0; s < sequences.size(); s++)
//                {
//                    errors[i][j][s] = errorsTmp[i][j][s];
//
//                    string id = "combination_" + to_str(i) + "-" + to_str(j) + "-" + to_str(s);
//                    fs << id << errorsTmp[i][j][s];
//                }
//            }
//
//            fs.release();
//
//            mntrCombinations = mntrPendingCombinations;
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
//            // Variables can be shared in some combinations (throughout those only the detection tolerance varies), do not need to recalculate
//            cv::Mat aux (mntrCombinations.row(0).size(), mntrCombinations.type()); // previous combination
//            vector<vector<pcl::PointXYZ> > detectionPositions; // previous detections
//
//            for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinations.rows; j++)
//            {
//                // special case: only changed the tolerance parameter on detections,
//                // no need to re-compute anything except errors (5th index, i.e. col or x == 4)
//                cv::Point p = cvx::diffIdx(mntrCombinations.row(j), aux);
//                if (p.x < 4)
//                {
//                    aux = mntrCombinations.row(j);
//
//                    pSys->getRegisterer()->setInputFrames(frames);
//                    pSys->getRegisterer()->registrate(frames);
//
//                    vector<cv::Mat> foregroundMasks;
//                    pSubtractors[i]->setInputFrames(frames);
//                    pSubtractors[i]->subtract(foregroundMasks);
//
//                    vector<cv::Mat> tabletopMasks;
//                    pSys->getTableModeler()->getTabletopMask(frames, tabletopMasks);
//
//                    for (int v = 0; v < pSeq->getNumOfViews(); v++)
//                        frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
//
//                    pObjectDetector->setMorhologyLevel(mntrCombinations.at<double>(j,0));
//                    pObjectDetector->setDownsamplingSize(mntrCombinations.at<double>(j,1));
//                    pObjectDetector->setClusteringIntradistanceFactor(mntrCombinations.at<double>(j,2));
//                    pObjectDetector->setMinClusterSize(mntrCombinations.at<double>(j,3));
//
//                    pObjectDetector->setInputFrames(frames);
//                    pObjectDetector->detect();
//                    pObjectDetector->getDetectionPositions(detectionPositions);
//                }
//
//                detectionGroundtruths[s].setTolerance(mntrCombinations.at<double>(j,4));
//
//                vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, rejections;
//                cv::Mat frameErrors;
//
//                detectionGroundtruths[s].getFrameSegmentationResults(pSeq->getFrameCounters(), detectionPositions, matches, rejections, frameErrors);
//
//                if (bQualitativeEvaluation)
//                    visualizeSegmentations(frames, matches, rejections, 0.02, 3);
//                else
//                    frameErrors.copyTo(errors[i][offset+j][s].col(f));
//            }
//
//            f++;
//            cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
//        }
//    }
//
//    if (!bQualitativeEvaluation)
//    {
//        cv::FileStorage fs;
//
//        boost::filesystem::path filePath (path + filename);
//        if (boost::filesystem::exists(filePath))
//        {
//            cv::Mat mntrCombinationsTmp;
//
//            fs.open(path + filename, cv::FileStorage::READ);
//            fs["mntr_combinations"] >> mntrCombinationsTmp;
//            fs.release();
//
//            offset = mntrCombinationsTmp.rows -  mntrCombinations.rows;
//        }
//
//        fs.open(path + filename, cv::FileStorage::APPEND);
//        for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinations.rows; j++)
//        {
//            for (int s = 0; s < sequences.size(); s++)
//            {
//                string id = "combination_" + to_str(i) + "-" + to_str(offset + j) + "-" + to_str(s);
//                fs << id << errors[i][offset+j][s];
//            }
//        }
//        fs.release();
//
//
//        boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
//        if (boost::filesystem::exists(fileBakPath))
//            boost::filesystem::remove(fileBakPath);
//    }
//}

// Faster version
void validateMonitorizationSegmentation2(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, cv::Mat& combinations, vector<vector<cv::Mat> >& errors, bool bQualitativeEvaluation)
{
    pSys->initialize();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = pSys->getBackgroundSequence();
    
    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = pBgSeq->getNumOfViews();
    for (int s = 0; s < sequences.size(); s++)
        assert (numOfViews == sequences[s]->getNumOfViews());
    // -------------------------------------------------
    
    cv::Mat mntrCombinations;
    expandParameters<double>(mntrParameters, mntrCombinations);
    cvx::combine(bsCombinations, mntrCombinations, 1, combinations);
    
    // Initialization of visualizer (qualitative) or data structures (quantitative)
    
    int offset = 0; // in case quantitatively evaluate and load previous results from file
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;
        
        // Backup file
        
        boost::filesystem::path filePath (path + filename);
        if ( !boost::filesystem::exists(filePath) ) // file doesnt exist
        {
            fs.open(path + filename, cv::FileStorage::WRITE);
            fs << "combinations" << combinations;
            fs.release();
            
            errors.resize(combinations.rows);
            for (int i = 0; i < combinations.rows; i++)
            {
                errors[i].resize(sequences.size());
                for (int s = 0; s < sequences.size(); s++)
                {
                    errors[i][s].create(sequences[s]->getNumOfViews(), sequences[s]->getMinNumOfFrames(), CV_32SC3);
                }
            }
        }
        else
        {
            boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
            boost::filesystem::copy_file(filePath, fileBakPath, boost::filesystem::copy_option::overwrite_if_exists);
            
            cv::Mat combinationsTmp;
            vector<vector<cv::Mat> > errorsTmp;
            loadMonitorizationSegmentationValidationFile2(path + filename, combinationsTmp, errorsTmp);
            
            offset = combinationsTmp.rows;
            
            cv::Mat matchedCombinations, pendingCombinations;
            cvx::match(combinationsTmp, combinations, matchedCombinations, pendingCombinations);
            
            fs.open(path + filename, cv::FileStorage::WRITE);
            
            if (pendingCombinations.empty())
            {
                fs << "combinations" << combinationsTmp;
            }
            else
            {
                cv::Mat aux;
                cv::vconcat(combinationsTmp, pendingCombinations, aux);
                fs << "combinations" << aux;
            }
            
            // Data structures
            errors.resize(offset + pendingCombinations.rows);
            for (int i = 0; i < offset + pendingCombinations.rows; i++)
            {
                errors[i].resize(sequences.size());
                for (int s = 0; s < sequences.size(); s++)
                {
                    errors[i][s].create(sequences[s]->getNumOfViews(), sequences[s]->getMinNumOfFrames(), CV_32SC3);
                }
            }
            
            for (int i = 0; i < offset; i++) for (int s = 0; s < sequences.size(); s++)
            {
                errors[i][s] = errorsTmp[i][s];
                
                string id = "combination_" + to_str(i) + "-" + to_str(s);
                fs << id << errorsTmp[i][s];
            }
            
            fs.release();
            
            combinations = pendingCombinations;
        }
        
    }
    
    if (!combinations.empty())
    {
        // Do not repeat the bs every time. Pre-compute the required combinations!
        cv::Mat bsPendingCombinations, bsPendingIndices;
        cvx::unique(combinations.colRange(0,bsCombinations.cols), 0, bsPendingCombinations, bsPendingIndices); // separate (bs,sgmn) parameters in combinations and find unique bs' sub-combinations
        
        vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors;
        pSubtractors.resize(bsPendingCombinations.rows);
        for (int i = 0; i < bsPendingCombinations.rows; i++)
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
        
        ObjectDetector::Ptr pObjectDetector = pSys->getObjectDetector();
        
        
        // Calculate the errors
        
        for (int s = 0; s < sequences.size(); s++)
        {
            Sequence<ColorDepthFrame>::Ptr pSeq = sequences[s];
            
            int f = 0;
            pSeq->restart();
            while (pSeq->hasNextFrames())
            {
                boost::timer t;
                vector<ColorDepthFrame::Ptr> frames = pSeq->nextFrames();
                
                pSys->getRegisterer()->setInputFrames(frames);
                pSys->getRegisterer()->registrate(frames);
                
                // Variables can be shared in some combinations (throughout those only the detection tolerance varies), do not need to recalculate
                cv::Mat prevCombination (combinations.row(0).size(), combinations.type()); // previous combination
                vector<vector<pcl::PointXYZ> > detectionPositions; // previous detections
                
                for (int i = 0; i < combinations.rows; i++)
                {
                    // special case: only changed the tolerance parameter on detections,
                    // no need to re-compute anything except errors (5th index, i.e. col or x == 4)
                    cv::Point p = cvx::diffIdx(combinations.row(i), prevCombination);
                    if ( p.x < (bsCombinations.cols + mntrCombinations.cols - 1) )
                    {
                        prevCombination = combinations.row(i);
                        
                        vector<cv::Mat> foregroundMasks;
                        pSubtractors[bsPendingIndices.at<int>(i,0)]->setInputFrames(frames);
                        pSubtractors[bsPendingIndices.at<int>(i,0)]->subtract(foregroundMasks);
                        
                        vector<cv::Mat> tabletopMasks;
                        pSys->getTableModeler()->getTabletopMask(frames, tabletopMasks);
                        
                        for (int v = 0; v < pSeq->getNumOfViews(); v++)
                            frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
                        
                        pObjectDetector->setDownsamplingSize(combinations.at<double>(i,bsCombinations.cols + 0));
                        pObjectDetector->setClusteringIntradistanceFactor(combinations.at<double>(i,bsCombinations.cols + 1));
                        pObjectDetector->setMinClusterSize(combinations.at<double>(i,bsCombinations.cols + 2));
                        
                        pObjectDetector->setInputFrames(frames);
                        pObjectDetector->detect();
                        pObjectDetector->getDetectionPositions(detectionPositions);
                    }
                    
                    detectionGroundtruths[s].setTolerance(combinations.at<double>(i,bsCombinations.cols + 3));
                    
                    vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, rejections;
                    cv::Mat frameErrors;
                    detectionGroundtruths[s].getFrameSegmentationResults(pSeq->getFrameCounters(), detectionPositions, matches, rejections, frameErrors);
                    
                    if (bQualitativeEvaluation)
                        visualizeSegmentations(frames, matches, rejections, 0.02, 3);
                    else
                        frameErrors.copyTo(errors[i+offset][s].col(f));
                }
                
                f++;
                cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
            }
        }
    }
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;
        fs.open(path + filename, cv::FileStorage::APPEND);
        for (int i = 0; i < combinations.rows; i++) for (int s = 0; s < sequences.size(); s++)
        {
            string id = "combination_" + to_str(i+offset) + "-" + to_str(s);
            fs << id << errors[i+offset][s];
        }
        fs.release();
        
        boost::filesystem::path filePath (path + filename);
        boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
        if (boost::filesystem::exists(fileBakPath))
            boost::filesystem::remove(fileBakPath);
    }
}

void summarizeMonitorizationSegmentationValidation(cv::Mat combinations, vector<vector<cv::Mat> > errors, void (*f)(cv::Mat, cv::Mat, cv::Mat, cv::Mat&), cv::Mat& meanScores, cv::Mat& sdScores)
{
    meanScores.create(combinations.rows, errors[0][0].rows, cv::DataType<float>::type);
    sdScores.create(combinations.rows, errors[0][0].rows, cv::DataType<float>::type);
    
    for (int i = 0; i < combinations.rows; i++)
    {
        cv::Mat seqErrors;
        cvx::hconcat(errors[i], seqErrors);
        
        // Calculate erros
        vector<cv::Mat> errorsChannels;
        cv::split(seqErrors, errorsChannels);
        
        cv::Mat scores;
        (*f)(errorsChannels[0], errorsChannels[1], errorsChannels[2], scores);
        
        for (int v = 0; v < seqErrors.rows; v++)
        {
            cv::Scalar mean, stddev;
            cv::meanStdDev(scores.row(v), mean, stddev);
            meanScores.at<float>(i, v)  =   mean.val[0];
            sdScores.at<float>(i, v)    = stddev.val[0];
        }
    }
}

void visualizeSegmentations(vector<ColorDepthFrame::Ptr> frames, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > rejections, float markersRadius, float lineWidth)
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
        for (int i = 0; i < matches[v].size(); i++)
        {
            p = matches[v][i].first;
            q = matches[v][i].second;
            
            pVis->addSphere(p, markersRadius, 0, 1, 0, "matches_prediction_" + to_str(v) + to_str(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, 1, 1, 1, "matches_groundtruth_"  + to_str(v) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_groundtruth_" + to_str(v) + to_str(i), viewports[v]);
            
            pVis->addLine(p, q, 0, 1, 0, "matches_line_" + to_str(v) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_line_" + to_str(v) + to_str(i), viewports[v]);
        }
        // Rejections are in red linked with red line to groundtruth
        for (int i = 0; i < rejections[v].size(); i++)
        {
            p = rejections[v][i].first;
            q = rejections[v][i].second;
            
            pVis->addSphere(p, markersRadius, 1, 0, 0, "rejections_prediction_" + to_str(v) + to_str(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, 1, 1, 1, "rejections_groundtruth_" + to_str(v) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_groundtruth_" + to_str(v) + to_str(i), viewports[v]);
            
            pVis->addLine(p, q, 1, 0, 0, "rejections_line_" + to_str(v) + to_str(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_line_" + to_str(v) + to_str(i), viewports[v]);
            
        }
    }
    
    pVis->spin();
}