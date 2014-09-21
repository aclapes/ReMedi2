//
//  validation.cpp
//  remedi2
//
//  Created by Albert Clapés on 30/08/14.
//
//

#include "ReMedi.h"
#include "constants.h"
#include "statistics.h"
#include "KinectReader.h"
#include "ObjectModel.hpp"

#include "cvxtended.h"
#include "conversion.h"

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <boost/timer.hpp>

// Declarations

// Background subtraction validation and performance
int loadBackgroundSubtractionValidationFile(string filePath, cv::Mat& combinations, vector<cv::Mat>& overlaps);
void validateBackgroundSubtraction(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr > sequences, vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename, cv::Mat& combinations, cv::Mat& overlaps);
void summarizeBackgroundSubtractionValidation(cv::Mat combinations, vector<cv::Mat> overlaps, cv::Mat& means, cv::Mat& stddevs);
//void showBsParametersPerformance(vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename);
//void getBsParametersPerformance(cv::Mat overlaps, cv::Mat& means, cv::Mat& stddevs);

// ObjectDetector segmentation validation and performance
void loadMonitorizationSegmentationValidationFile(string filePath, cv::Mat& bsCombinations, cv::Mat& mntrCombinations, vector<vector<vector<cv::Mat> > >& errors);
void validateMonitorizationSegmentation(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);
void summarizeMonitorizationSegmentationValidation(cv::Mat bsCombinations, cv::Mat mntrCombinations, vector<vector<vector<cv::Mat> > > errors, void (*f)(cv::Mat, cv::Mat, cv::Mat, cv::Mat&), cv::Mat& combinations, cv::Mat& meanScores, cv::Mat& sdScores);

// ObjectDetector recognition performance
void validateMonitorizationRecognition(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat combinations, vector<vector<double> > rcgnParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

// General functions
void showValidationSummary(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat meanScores, cv::Mat sdScores, bool bMinimize = false);
void getBestCombinations(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, int k, cv::Mat scores, cv::Mat& bestCombinations, bool bMinimize = false);
int validation();


int loadBackgroundSubtractionValidationFile(string path, cv::Mat& combinations, vector<cv::Mat>& overlaps)
{
    combinations.release();
    overlaps.clear();
    
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    
    if (!fs.isOpened()) // could not read error
        return -1;
    
    fs["combinations"] >> combinations;
    
    overlaps.resize(combinations.rows);
    for (int c = 0; c < combinations.rows; c++)
        fs["overlaps_" + to_string(c)] >> overlaps[c];
    
    fs.release();
    
    return 0;
}

void validateBackgroundSubtraction(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr > sequences, vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename, cv::Mat& combinations, vector<cv::Mat>& overlaps)
{
    sys.initialize();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = sys.getBackgroundSequence();
    
    // Some consistency checks

    int numOfViews = pBgSeq->getNumOfViews();
    int numOfForegroundMasks = fgMasksSequences[0]->getMinNumOfFrames();
    
    for (int s = 0; s < sequences.size(); s++)
        assert (numOfViews == sequences[s]->getNumOfViews());
        
    for (int s = 0; s < fgMasksSequences.size(); s++)
    {
        assert (numOfViews == fgMasksSequences[s]->getNumOfViews());
        for (int v = 0; v < numOfViews; v++)
            assert (numOfForegroundMasks == fgMasksSequences[s]->getMinNumOfFrames());
    }
    
    // Backup file
    
    boost::filesystem::path filePath (path + filename);
    boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
    if (boost::filesystem::exists(filePath))
        boost::filesystem::copy_file(filePath, fileBakPath, boost::filesystem::copy_option::overwrite_if_exists);
    
    // Create subtractor
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs = sys.getBackgroundSubtractor();
    pBs->setFramesResolution(X_RESOLUTION, Y_RESOLUTION);
    pBs->setBackgroundSequence(pBgSeq, BS_NUM_OF_SAMPLES);

    // Prepare combinations of parameters to test
    
    expandParameters<double>(parameters, combinations);

    // New combinations to compute? Check which are they
    
    cv::Mat combinationsTmp;
    std::vector<cv::Mat> overlapsTmp;
    string filepath = path + filename;
    loadBackgroundSubtractionValidationFile(filepath, combinationsTmp, overlapsTmp);
    
    cv::Mat matched, pending;
    cvx::match(combinations, combinationsTmp, matched, pending);
    
    // Create a file to store the previous combinations' results and the new (if needed)
    
    cv::FileStorage fs;
    fs.open(path + filename, cv::FileStorage::WRITE);
    
    if (pending.empty()) combinations = combinationsTmp;
    else cv::vconcat(combinationsTmp, pending, combinations);
    
    overlaps = overlapsTmp;
    
    fs << "combinations" << combinations;
    for (int c = 0; c < combinationsTmp.rows; c++)
    {
        fs << "overlaps_" + to_string(c) << overlapsTmp[c];
    }
    
    fs.release();
    
    // Compute the new
    
    for (int c = 0; c < pending.rows; c++)
    {
        string parametersStr = to_string_with_precision<double>(pending.row(c), "_", 4);
        cout << c << " : " << parametersStr << endl;
        
        pBs->setModality(pending.at<double>(c,0));
        pBs->setNumOfMixtureComponents(pending.at<double>(c,1));
        pBs->setLearningRate(pending.at<double>(c,2));
        pBs->setBackgroundRatio(pending.at<double>(c,3));
        pBs->setVarThresholdGen(pending.at<double>(c,4));
        pBs->setOpeningSize(pending.at<double>(c,5));
        
        pBs->model();
        
        vector<cv::Mat> seqsOverlaps;
        
        for (int s = 0; s < fgMasksSequences.size(); s++)
        {
            Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
            Sequence<ColorDepthFrame>::Ptr pTestSeq = sequences[s];
            
            string seqPath = path + to_string(s+1) + "/";
            
            cv::Mat seqOverlaps (pFgGtSeq->getNumOfViews(), pFgGtSeq->getMinNumOfFrames(), cv::DataType<float>::type);
            
            int f = 0;
            pFgGtSeq->restart();
            while (pFgGtSeq->hasNextFrames())
            {
                vector<Frame::Ptr> fgGtMasksFrames = pFgGtSeq->nextFrames();
                
                // search for correspondence using filename
                vector<string> filenames = pFgGtSeq->getFramesFilenames();
                vector<ColorDepthFrame::Ptr> testFrames = pTestSeq->getFrames(filenames);
                sys.getRegisterer()->setInputFrames(testFrames);
                sys.getRegisterer()->registrate(testFrames);
                
                // focus only on tabletop's region for the BS
                vector<cv::Mat> tabletopMasks;
                sys.getTableModeler()->getTabletopMask(testFrames, tabletopMasks);
                
                // test
                vector<cv::Mat> foregroundMasks;
                pBs->setInputFrames(testFrames);
                pBs->subtract(foregroundMasks);
                
                // gather the test results
                for (int v = 0; v < pBgSeq->getNumOfViews(); v++)
                {
                    // Masks were in disk as RGB images (preprocessing)
                    cv::Mat fgGtGrayMask, fgGtBinMask;
                    cv::cvtColor(fgGtMasksFrames[v]->get(), fgGtGrayMask, CV_BGR2GRAY);
                    cv::threshold(fgGtGrayMask, fgGtBinMask, 0, 255, cv::THRESH_BINARY);
                    
                    cv::Mat maskedPrediction = foregroundMasks[v] & tabletopMasks[v];
                    
                    // Quantitative result (overlap)
                    seqOverlaps.at<float>(v,f++) = cvx::overlap(maskedPrediction, fgGtBinMask);
                    
                    // Qualitative result (mask image)
                    string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                    cv::imwrite(maskPath + filenames[v] + "-" + parametersStr + ".png", maskedPrediction);
                }
            }
            
            seqsOverlaps.push_back(seqOverlaps);
        }
        
        cv::Mat combinationOverlaps;
        cvx::vconcat(seqsOverlaps, combinationOverlaps);
        
        fs.open(path + filename, cv::FileStorage::APPEND);
        fs << "overlaps_" + to_string(c) << combinationOverlaps;
        fs.release();
        
        overlaps.push_back(combinationOverlaps);
    }
    
    if (boost::filesystem::exists(fileBakPath))
        boost::filesystem::remove(fileBakPath);
}

void summarizeBackgroundSubtractionValidation(cv::Mat combinations, vector<cv::Mat> overlaps, cv::Mat& means, cv::Mat& stddevs)
{
    assert (combinations.rows == overlaps.size());
    
    means.create(overlaps.size(), overlaps[0].rows, cv::DataType<float>::type);
    stddevs.create(overlaps.size(), overlaps[0].rows, cv::DataType<float>::type);
    for (int c = 0; c < overlaps.size(); c++)
    {
        assert (overlaps[0].rows == overlaps[c].rows);
        for (int v = 0; v < overlaps[c].rows; v++)
        {
            cv::Scalar m, s;
            cv::meanStdDev(overlaps[c].row(v), m, s);
            means.at<float>(c,v) = m.val[0];
            stddevs.at<float>(c,v) = s.val[0];
        }
    }
}

void showBsParametersPerformance(vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename)
{
//    cv::Mat combinations, overlaps;
//    
//    cv::FileStorage fs;
//    fs.open(path + filename, cv::FileStorage::READ);
//    fs["combinations"] >> combinations;
//    fs["overlaps"] >> overlaps;
//    fs.release();
//    
//    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
//    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
//    {
//        cv::Rect roi (v * numOfFrames, 0, numOfFrames, combinations.rows);
//        cv::Mat overlapsView (overlaps, roi);
//        
//        cv::Mat overlapAvgView;
//        cv::reduce(overlapsView, overlapAvgView, 1, CV_REDUCE_AVG);
//        
//        cv::Mat I;
//        cv::sortIdx(overlapAvgView, I, cv::SORT_EVERY_COLUMN | cv::SORT_DESCENDING);
//        
//        int best = 10;
//        for (int i = 0; i < best; i++)
//        {
//            int idx = I.at<int>(0,i);
//            cout << i << "/" << best << combinations.row(idx) << "\t" << overlapAvgView.row(idx) << endl;
//            
//            for (int s = 3; s < 4 /*fgMasksSequences.size()*/; s++)
//            {
//                string seqPath = path + to_string(s+1) + "/";
//                string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
//                
//                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
//                pFgGtSeq->restart();
//                //while (pFgGtSeq->hasNextFrames())
//                //{
//                vector<Frame::Ptr> frames = pFgGtSeq->nextFrames();
//                vector<string> filenames = pFgGtSeq->getFramesFilenames();
//                
//                cout << v << " " << i << " " << s << " " << filenames[v] << endl;
//                
//                string filename = filenames[v] + "-"; // then concantenated with params combination
//                filename += to_string_with_precision(combinations.at<double>(idx,0), 4);
//                for (int p = 1; p < combinations.cols; p++)
//                    filename += "_" + to_string_with_precision(combinations.at<double>(idx,p), 4);
//                
//                cv::Mat maskedPrediction;
//                maskedPrediction = cv::imread(maskPath + filename + ".png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_UNCHANGED);
//                
//                cv::imshow("loaded", maskedPrediction);
//                cv::waitKey(0);
//                //}
//            }
//        }
//        cout << endl;
//    }
//    cout << endl;
}

void getBsParametersPerformance(cv::Mat overlaps, cv::Mat& means, cv::Mat& stddevs)
{
//        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs = sys.getBackgroundSubtractor();
//    means.create(overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
//    stddevs.create(overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
//    
//    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
//    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
//    {
//        cv::Rect roi (v * numOfFrames, 0, numOfFrames, overlaps.rows);
//        cv::Mat overlapsView (overlaps, roi);
//        
//        for (int i = 0; i < overlaps.rows; i++)
//        {
//            cv::Scalar m, s;
//            cv::meanStdDev(overlapsView.row(i), m, s);
//            means.at<float>(i,v) = m.val[0];
//            stddevs.at<float>(i,v) = s.val[0];
//        }
//    }
}

void loadMonitorizationSegmentationValidationFile(string filePath, cv::Mat& bsCombinations, cv::Mat& mntrCombinations, vector<vector<vector<cv::Mat> > >& errors)
{
    cv::FileStorage fs;
    fs.open(filePath, cv::FileStorage::READ);
    fs["bs_combinations"] >> bsCombinations;
    fs["mntr_combinations"] >> mntrCombinations;
    
    errors.resize(bsCombinations.rows);
    for (int i = 0; i < bsCombinations.rows; i++)
    {
        errors[i].resize(mntrCombinations.rows);
        for (int j = 0; j < mntrCombinations.rows; j++)
        {
            cv::Mat m;
            
            int s = 0;
            do
            {
                string str = "combination_" + to_string(i) + "-" + to_string(j) + "-" + to_string(s++);
                fs[str.c_str()] >> m;
                
                if (!m.empty()) errors[i][j].push_back(m);
            }
            while (!m.empty());
            
        }
    }
    
    fs.release();
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
        pVis->addCoordinateSystem(0.1, 0, 0, 0, "cs" + to_string(v), viewports[v]);
    }
    
    // Draw clouds, matches, and rejections
    
    for (int v = 0; v < frames.size(); v++)
    {
        // Annotations are not registered, so get the unregistered point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColoredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        frames[v]->getColoredPointCloud( *pColoredCloud ); // unregistered
        
        pVis->addPointCloud(pColoredCloud, "cloud_" + to_string(v), viewports[v]);
        
        pcl::PointXYZ p, q;
        
        // Groundtruth annotations represented as yellow spheres
        // matches are in green linked with green line to groundtruth
        for (int i = 0; i < matches[v].size(); i++)
        {
            p = matches[v][i].first;
            q = matches[v][i].second;
            
            pVis->addSphere(p, markersRadius, 0, 1, 0, "matches_prediction_" + to_string(v) + to_string(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, 1, 1, 1, "matches_groundtruth_"  + to_string(v) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_groundtruth_" + to_string(v) + to_string(i), viewports[v]);
            
            pVis->addLine(p, q, 0, 1, 0, "matches_line_" + to_string(v) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_line_" + to_string(v) + to_string(i), viewports[v]);
        }
        // Rejections are in red linked with red line to groundtruth
        for (int i = 0; i < rejections[v].size(); i++)
        {
            p = rejections[v][i].first;
            q = rejections[v][i].second;
            
            pVis->addSphere(p, markersRadius, 1, 0, 0, "rejections_prediction_" + to_string(v) + to_string(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, 1, 1, 1, "rejections_groundtruth_" + to_string(v) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_groundtruth_" + to_string(v) + to_string(i), viewports[v]);
            
            pVis->addLine(p, q, 1, 0, 0, "rejections_line_" + to_string(v) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_line_" + to_string(v) + to_string(i), viewports[v]);
            
        }
    }
    
    pVis->spin();
}

// Faster version
void validateMonitorizationSegmentation(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation)
{
    sys.initialize();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = sys.getBackgroundSequence();

    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = pBgSeq->getNumOfViews();
    for (int s = 0; s < sequences.size(); s++)
        assert (numOfViews == sequences[s]->getNumOfViews());
    // -------------------------------------------------
    
    cv::Mat mntrCombinations;
    expandParameters<double>(mntrParameters, mntrCombinations);
    
    vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors;
    pSubtractors.resize(bsCombinations.rows);
    for (int i = 0; i < bsCombinations.rows; i++)
    {
        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs (new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
        *pBs = *(sys.getBackgroundSubtractor());
        
        pBs->setModality(bsCombinations.at<double>(i,0));
        pBs->setNumOfMixtureComponents(bsCombinations.at<double>(i,1));
        pBs->setLearningRate(bsCombinations.at<double>(i,2));
        pBs->setBackgroundRatio(bsCombinations.at<double>(i,3));
        pBs->setVarThresholdGen(bsCombinations.at<double>(i,4));
        pBs->setOpeningSize(bsCombinations.at<double>(i,5));
        
        pBs->model();
        
        pSubtractors[i] = pBs;
    }
    
    ObjectDetector::Ptr pObjectDetector = sys.getObjectDetector();
    
    // Initialization of visualizer (qualitative) or data structures (quantitative)
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;

        // Backup file
        
        boost::filesystem::path filePath (path + filename);
        if (!boost::filesystem::exists(filePath))
        {
            fs.open(path + filename, cv::FileStorage::WRITE);
            fs << "bs_combinations" << bsCombinations;
            fs << "mntr_combinations" << mntrCombinations;
            fs.release();
        }
        else
        {
            boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
            boost::filesystem::copy_file(filePath, fileBakPath, boost::filesystem::copy_option::overwrite_if_exists);
        
            cv::Mat bsCombinationsTmp, mntrCombinationsTmp;
            vector<vector<vector<cv::Mat> > > errorsTmp;
            loadMonitorizationSegmentationValidationFile(path + filename, bsCombinationsTmp, mntrCombinationsTmp, errorsTmp);
            
            cv::Mat mntrMatchedCombinations, mntrPendingCombinations;
            cvx::match(mntrCombinationsTmp, mntrCombinations, mntrMatchedCombinations, mntrPendingCombinations);
            
            fs.open(path + filename, cv::FileStorage::WRITE);
            
            fs << "bs_combinations" << bsCombinations;
            
            if (mntrPendingCombinations.empty())
            {
                fs << "mntr_combinations" << mntrCombinationsTmp;
            }
            else
            {
                cv::Mat aux;
                cv::vconcat(mntrCombinationsTmp, mntrPendingCombinations, aux);
                fs << "mntr_combinations" << aux;
            }
        
            for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinationsTmp.rows; j++)
            {
                for (int s = 0; s < sequences.size(); s++)
                {
                    string id = "combination_" + to_string(i) + "-" + to_string(j) + "-" + to_string(s);
                    fs << id << errorsTmp[i][j][s];
                }
            }
            
            fs.release();
            
            mntrCombinations = mntrPendingCombinations;
        }
        
        // Data structures
        
        errors.resize(bsCombinations.rows);
        for (int i = 0; i < bsCombinations.rows; i++)
        {
            errors[i].resize(mntrCombinations.rows);
            for (int j = 0; j < mntrCombinations.rows; j++)
            {
                errors[i][j].resize(sequences.size());
                for (int s = 0; s < sequences.size(); s++)
                {
                    errors[i][j][s].create(sequences[s]->getNumOfViews(), sequences[s]->getMinNumOfFrames(), CV_32SC3);
                }
            }
        }
    }
    
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

            // Variables can be shared in some combinations (throughout those only the detection tolerance varies), do not need to recalculate
            cv::Mat aux (mntrCombinations.row(0).size(), mntrCombinations.type()); // previous combination
            vector<vector<pcl::PointXYZ> > detectionPositions; // previous detections
            
            for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinations.rows; j++)
            {
                // special case: only changed the tolerance parameter on detections,
                // no need to re-compute anything except errors (5th index, i.e. col or x == 4)
                cv::Point p = cvx::diffIdx(mntrCombinations.row(j), aux);
                if (p.x < 4)
                {
                    aux = mntrCombinations.row(j);
                    
                    sys.getRegisterer()->setInputFrames(frames);
                    sys.getRegisterer()->registrate(frames);
                    
                    vector<cv::Mat> foregroundMasks;
                    pSubtractors[i]->setInputFrames(frames);
                    pSubtractors[i]->subtract(foregroundMasks);
                    
                    vector<cv::Mat> tabletopMasks;
                    sys.getTableModeler()->getTabletopMask(frames, tabletopMasks);
                    
                    for (int v = 0; v < pSeq->getNumOfViews(); v++)
                        frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
                    
                    pObjectDetector->setMorhologyLevel(mntrCombinations.at<double>(j,0));
                    pObjectDetector->setDownsamplingSize(mntrCombinations.at<double>(j,1));
                    pObjectDetector->setClusteringIntradistanceFactor(mntrCombinations.at<double>(j,2));
                    pObjectDetector->setMinClusterSize(mntrCombinations.at<double>(j,3));
                    
                    pObjectDetector->setInputFrames(frames);
                    pObjectDetector->detect();
                    pObjectDetector->getDetectionPositions(detectionPositions);
                }
                
                detectionGroundtruths[s].setTolerance(mntrCombinations.at<double>(j,4));
                
                vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, rejections;
                cv::Mat frameErrors;
                
                detectionGroundtruths[s].getFrameSegmentationResults(pSeq->getFrameCounters(), detectionPositions, matches, rejections, frameErrors);
                
                if (bQualitativeEvaluation)
                    visualizeSegmentations(frames, matches, rejections, 0.02, 3);
                else
                    frameErrors.copyTo(errors[i][j][s].col(f));
            }
            
            f++;
            cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
        }
    }
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;
        
        int offset = 0;
        
        boost::filesystem::path filePath (path + filename);
        if (boost::filesystem::exists(filePath))
        {
            cv::Mat mntrCombinationsTmp;

            fs.open(path + filename, cv::FileStorage::READ);
            fs["mntr_combinations"] >> mntrCombinationsTmp;
            fs.release();
        
            offset = mntrCombinationsTmp.rows -  mntrCombinations.rows;
        }
        
        fs.open(path + filename, cv::FileStorage::APPEND);
        for (int i = 0; i < bsCombinations.rows; i++) for (int j = 0; j < mntrCombinations.rows; j++)
        {
            for (int s = 0; s < sequences.size(); s++)
            {
                string id = "combination_" + to_string(i) + "-" + to_string(offset + j) + "-" + to_string(s);
                fs << id << errors[i][j][s];
            }
        }
        fs.release();
        
        
        boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
        if (boost::filesystem::exists(fileBakPath))
            boost::filesystem::remove(fileBakPath);
    }
}

void summarizeMonitorizationSegmentationValidation(cv::Mat bsCombinations, cv::Mat mntrCombinations, vector<vector<vector<cv::Mat> > > errors, void (*f)(cv::Mat, cv::Mat, cv::Mat, cv::Mat&), cv::Mat& combinations, cv::Mat& meanScores, cv::Mat& sdScores)
{
    combinations.create(bsCombinations.rows * mntrCombinations.rows, bsCombinations.cols + mntrCombinations.cols, cv::DataType<double>::type);
    meanScores.create(bsCombinations.rows * mntrCombinations.rows, errors[0][0][0].rows, cv::DataType<float>::type);
    sdScores.create(bsCombinations.rows * mntrCombinations.rows, errors[0][0][0].rows, cv::DataType<float>::type);
    
    for (int i = 0; i < bsCombinations.rows; i++)
    {
        for (int j = 0; j < mntrCombinations.rows; j++)
        {
            // Concatenate combinations
            cv::Mat combinationsRow;
            cv::hconcat(bsCombinations.row(i), mntrCombinations.row(j), combinationsRow);
            combinationsRow.copyTo( combinations.row(i * mntrCombinations.rows + j) );
            
            cv::Mat seqErrors;
            cvx::hconcat(errors[i][j], seqErrors);
            
            // Calculate erros
            vector<cv::Mat> errorsChannels;
            cv::split(seqErrors, errorsChannels);
            
            cv::Mat scores;
            (*f)(errorsChannels[0], errorsChannels[1], errorsChannels[2], scores);
            
            for (int v = 0; v < seqErrors.rows; v++)
            {
                cv::Scalar mean, stddev;
                cv::meanStdDev(scores.row(v), mean, stddev);
                meanScores.at<float>(i * mntrCombinations.rows + j, v)  =   mean.val[0];
                sdScores.at<float>(i * mntrCombinations.rows + j, v)    = stddev.val[0];
            }
        }
    }
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
        pVis->addCoordinateSystem(0.1, 0, 0, 0, "cs" + to_string(v), viewports[v]);
    }
    
    // Draw clouds, matches, and rejections
    
    for (int v = 0; v < frames.size(); v++)
    {
        // Annotations are not registered, so get the unregistered point clouds
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pColoredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        frames[v]->getColoredPointCloud( *pColoredCloud ); // unregistered
        
        pVis->addPointCloud(pColoredCloud, "cloud_" + to_string(v), viewports[v]);
        
        pcl::PointXYZ p, q;
        
        // Groundtruth annotations represented as yellow spheres
        // matches are in green linked with green line to groundtruth
        vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > objects = matches[v];
        for (int o = 0; o < matches[v].size(); o++) for (int i = 0; i < matches[v][o].size(); i++)
        {
            p = matches[v][o][i].first;
            q = matches[v][o][i].second;
            
            pVis->addSphere(p, markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "matches_prediction_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "matches_groundtruth_"  + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_groundtruth_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            
            pVis->addLine(p, q, 0, 1, 0, "matches_line_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "matches_line_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
        }
        // Rejections are in red linked with red line to groundtruth
        for (int o = 0; o < rejections[v].size(); o++)  for (int i = 0; i < rejections[v][o].size(); i++)
        {
            p = rejections[v][o][i].first;
            q = rejections[v][o][i].second;
            
            pVis->addSphere(p, markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_prediction_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            
            pVis->addCube(q.x - markersRadius, q.x + markersRadius, q.y - markersRadius, q.y + markersRadius, q.z - markersRadius, q.z + markersRadius, g_Colors[o][2], g_Colors[o][1], g_Colors[o][0], "rejections_groundtruth_"  + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_groundtruth_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            
            pVis->addLine(p, q, 1, 0, 0, "rejections_line_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
            pVis->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, "rejections_line_" + to_string(v) + to_string(o) + to_string(i), viewports[v]);
       }
    }
    
    pVis->spin();
}

// Faster version
void validateMonitorizationRecognition(ReMedi sys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat combinations, vector<vector<double> > rcgnParameters, vector<DetectionOutput> detectionGroundtruths, string path, string filename, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation)
{
    sys.initialize();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = sys.getBackgroundSequence();
    
    // Some consistency checks
    // -------------------------------------------------
    int numOfViews = pBgSeq->getNumOfViews();
    for (int s = 0; s < sequences.size(); s++)
        assert (numOfViews == sequences[s]->getNumOfViews());
    // -------------------------------------------------
    
    cv::Mat rcgnCombinations;
    expandParameters<double>(rcgnParameters, rcgnCombinations);
    
    // Do not repeat the bs every time. Pre-compute the required combinations!
    cv::Mat bsCombinations, bsIndices;
    cvx::unique(combinations.colRange(0,6), 0, bsCombinations, bsIndices); // separate (bs,sgmn) parameters in combinations and find unique bs' sub-combinations
    
    vector<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr> pSubtractors (bsCombinations.rows);
    
    for (int i = 0; i < bsCombinations.rows; i++)
    {
        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs (new BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>);
        *pBs = *(sys.getBackgroundSubtractor());
        
        pBs->setModality(bsCombinations.at<double>(i,0));
        pBs->setNumOfMixtureComponents(bsCombinations.at<double>(i,1));
        pBs->setLearningRate(bsCombinations.at<double>(i,2));
        pBs->setBackgroundRatio(bsCombinations.at<double>(i,3));
        pBs->setVarThresholdGen(bsCombinations.at<double>(i,4));
        pBs->setOpeningSize(bsCombinations.at<double>(i,5));
        
        pBs->model();
    
        pSubtractors[i] = pBs;
    }
    
    // Same for object recognizer (and its internal cloudjectmodels)
    
    cv::Mat descCombinations, descIndices;
    cvx::unique(rcgnCombinations.colRange(0,1), 0, descCombinations, descIndices); // separate (descs,strategies) parameters in combinations and find unique bs' sub-combinations
    
    vector<void*> recognizers (descCombinations.rows);
    
    for (int i = 0; i < descCombinations.rows; i++)
    {
        int dType = (int) descCombinations.at<double>(i,0);
        if (dType == DESCRIPTION_FPFH)
        {
            if (sys.getDescriptionType() == dType)
                recognizers[i] = sys.getObjectRecognizer();
            else
            {
                ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>* pOR = new ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>();
                
                if (sys.getDescriptionType() == DESCRIPTION_PFHRGB)
                    *pOR = *((ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) sys.getObjectRecognizer());
                // else if (in case of more kinds of descriptions)
                // ...
                
                pOR->create();
                
                recognizers[i] = (void*) pOR;
            }
        }
        else if (dType == DESCRIPTION_PFHRGB)
        {
            if (sys.getDescriptionType() == dType)
                recognizers[i] = sys.getObjectRecognizer();
            else
            {
                ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>* pOR = new ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>();
                
                if (sys.getDescriptionType() == DESCRIPTION_FPFH)
                    *pOR = *((ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) sys.getObjectRecognizer());
                // else if (in case of more kinds of descriptions)
                // ...
                
                pOR->create();
                
                recognizers[i] = (void*) pOR;
            }
        }
    }
    
    // Initialization of visualizer (qualitative) or data structures (quantitative)
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;
        fs.open(path + filename, cv::FileStorage::WRITE);
        fs << "rcgn_combinations" << rcgnCombinations;
        fs << "bs-sgmn_combinations" << combinations;
        fs.release();
        
        // Data structures
        
        errors.resize(rcgnCombinations.rows);
        for (int i = 0; i < errors.size(); i++)
        {
            errors[i].resize(combinations.rows);
            for (int j = 0; j < errors[i].size(); j++)
            {
                errors[i][j].resize(sequences.size());
                for (int s = 0; s < errors[i][j].size(); s++)
                {
                    errors[i][j][s] = cv::Mat(sequences[s]->getNumOfViews(),
                                              sequences[s]->getMinNumOfFrames(),
                                              CV_32SC3, cv::Scalar(0));
                }
            }
        }
    
    }

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
            
            sys.getRegisterer()->setInputFrames(frames);
            sys.getRegisterer()->registrate(frames);
            
            for (int i = 0; i < rcgnCombinations.rows; i++)
            {
                vector<vector< pair<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > > detectionCorrespondences;
                vector<vector<vector<pcl::PointXYZ> > > recognitions; // previous recognitions
                
                for (int j = 0; j < combinations.rows; j++)
                {
                    vector<cv::Mat> foregroundMasks;
                    pSubtractors[bsIndices.at<int>(j,0)]->setInputFrames(frames);
                    pSubtractors[bsIndices.at<int>(j,0)]->subtract(foregroundMasks);
                    
                    vector<cv::Mat> tabletopMasks;
                    sys.getTableModeler()->getTabletopMask(frames, tabletopMasks);
                    
                    for (int v = 0; v < pSeq->getNumOfViews(); v++)
                    {
                        frames[v]->setMask(foregroundMasks[v] & tabletopMasks[v]);
                    }
                    
                    ObjectDetector::Ptr pObjectDetector = sys.getObjectDetector();
                    pObjectDetector->setInputFrames(frames);
                    
                    pObjectDetector->setMorhologyLevel(combinations.at<double>(j, bsCombinations.cols + 0));
                    pObjectDetector->setDownsamplingSize(combinations.at<double>(j, bsCombinations.cols + 1));
                    pObjectDetector->setClusteringIntradistanceFactor(combinations.at<double>(j, bsCombinations.cols + 2));
                    pObjectDetector->setMinClusterSize(combinations.at<double>(j, bsCombinations.cols + 3));
                    pObjectDetector->setInterviewCorrepondenceDistance(combinations.at<double>(j, bsCombinations.cols + 4));
                    
                    pObjectDetector->detect();
                    if ((int) rcgnCombinations.at<double>(i,1) == RECOGNITION_MONOCULAR)
                        pObjectDetector->getDetectionCorrespondences(detectionCorrespondences, false);
                    else
                        pObjectDetector->getDetectionCorrespondences(detectionCorrespondences, true);
                    
                    int dType = (int) rcgnCombinations.at<double>(i,0);
                    if (dType == DESCRIPTION_FPFH)
                    {
                        ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>* pOR = (ObjectRecognizer<pcl::PointXYZRGB,pcl::FPFHSignature33>*) recognizers[descIndices.at<int>(i,0)];
                        pOR->setInputDetections(detectionCorrespondences);
                        pOR->recognize(recognitions);
                    }
                    else if (dType == DESCRIPTION_PFHRGB)
                    {
                        ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>* pOR = (ObjectRecognizer<pcl::PointXYZRGB,pcl::PFHRGBSignature250>*) recognizers[descIndices.at<int>(i,0)];
                        pOR->setInputDetections(detectionCorrespondences);
                        pOR->recognize(recognitions);
                    }
                    
                    detectionGroundtruths[s].setTolerance(combinations.at<double>(j, bsCombinations.cols + 4));
                    
                    vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, rejections;
                    cv::Mat frameErrors;
                    detectionGroundtruths[s].getFrameRecognitionResults(pSeq->getFrameCounters(), recognitions, matches, rejections, frameErrors);
                    
                    if (bQualitativeEvaluation)
                        visualizeRecognitions(frames, matches, rejections, 0.02, 3);
                    else
                        frameErrors.copyTo(errors[i][j][s].col(f));
                }
            }
            
            // DEBUG
            // <-----------------------------------------------------------------------------------------------
            for (int i = 0; i < rcgnCombinations.rows; i++) for (int j = 0; j < combinations.rows; j++)
            {
                cv::Mat errorsF;
                errors[i][j][s].convertTo(errorsF, CV_32FC(errors[i][j][s].channels()));
                cv::Mat errorsRoi (errorsF, cv::Rect(0, 0, f+1, errorsF.rows));
                
                cv::Mat errorsRoiReduced;
                cv::reduce(errorsRoi, errorsRoiReduced, 1, CV_REDUCE_SUM);
                
                cout << (i * combinations.rows + j) << " " << i << " " << j << " : ";
                for (int v = 0; v < errorsRoiReduced.rows; v++)
                {
                    cv::Vec3f err = errorsRoiReduced.at<cv::Vec3f>(v,0);
                    float fscore = computeF1Score(err.val[0], err.val[1], err.val[2]);
                    cout << (v > 0 ? ", " : "") << to_string_with_precision<float>(fscore);
                } cout << endl;
            }
            // ----------------------------------------------------------------------------------------------->
            
            f++;
            cout << "Processing the frame took " << t.elapsed() << " secs." << endl;
        }
    }
    
    if (!bQualitativeEvaluation)
    {
        cv::FileStorage fs;
        fs.open(path + filename, cv::FileStorage::APPEND);
        for (int i = 0; i < rcgnCombinations.rows; i++)
        {
            for (int j = 0; j < combinations.rows; j++)
            {
                for (int s = 0; s < sequences.size(); s++)
                {
                    string id = "combination_" + to_string(i) + "-" + to_string(j) + "-" + to_string(s);
                    fs << id << errors[i][j][s];
                }
            }
        }
        fs.release();
    }
}

void showValidationSummary(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat meanScores, cv::Mat sdScores, bool bMinimize)
{
    assert (combinations.rows == meanScores.rows);
    
    cv::Mat reducedMeanScores;
    cv::reduce(meanScores, reducedMeanScores, 1, CV_REDUCE_AVG);
    
    cv::Mat fcombinations;
    expandParameters<double>(parameters, fcombinations);
    
    for (int c = 0; c < fcombinations.rows; c++)
    {
        cv::Mat mask (combinations.rows, 1, cv::DataType<uchar>::type, cv::Scalar(255));
        for (int p = 0; p < fcombinations.cols; p++)
        {
            cv::Mat aux = ( combinations.col(indices[p]) == fcombinations.at<double>(c,p) );
            mask &= aux;
        }
        
        double minVal, maxVal;
        cv::Point minIdx, maxIdx;
        cv::minMaxLoc(reducedMeanScores, &minVal, &maxVal, &minIdx, &maxIdx, mask);
        
        float bestVal = bMinimize ? minVal : maxVal;
        cv::Point bestIdx = bMinimize ? minIdx : maxIdx;
        cout << combinations.row(bestIdx.y) << " : " << bestVal << endl;
        
        cout << "[";
        for (int v = 0; v < meanScores.cols; v++)
        {
            if (v > 0) cout << ", ";
            cout << to_string_with_precision(meanScores.at<float>(bestIdx.y, v), 3) + " ± " + to_string_with_precision(sdScores.at<float>(bestIdx.y, v), 3);
        }
        cout << "]" << endl;
    }
}

void getBestCombinations(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, int k, cv::Mat scores, cv::Mat& bestCombinations, bool bMinimize)
{
    cv::Mat means;
    cvx::harmonicMean(scores, means, 1);
    
    cv::Mat fcombinations;
    expandParameters<double>(parameters, fcombinations);
    
    bestCombinations.create(fcombinations.rows * k, combinations.cols, combinations.type());
    
    for (int c = 0; c < fcombinations.rows; c++)
    {
        cv::Mat mask (combinations.rows, 1, cv::DataType<uchar>::type, cv::Scalar(255));
        for (int p = 0; p < fcombinations.cols; p++)
        {
            cv::Mat aux = ( combinations.col(indices[p]) == fcombinations.at<double>(c,p) );
            mask &= aux;
        }
        
        cv::Mat indexedCombinations, indexedMeans;
        cvx::indexMat(combinations, indexedCombinations, mask);
        cvx::indexMat(means, indexedMeans, mask);
        
        cv::Mat I;
        cv::sortIdx( indexedMeans, I, (bMinimize ? CV_SORT_ASCENDING : CV_SORT_DESCENDING) | CV_SORT_EVERY_COLUMN );
        
        for (int i = 0; i < k; i++)
            indexedCombinations.row(I.at<int>(i,0)).copyTo( bestCombinations.row(c * k + i) );
    }
}

int validation()
{
    vector<string> colorDirNames, depthDirNames;
    colorDirNames += COLOR_DIRNAME_1, COLOR_DIRNAME_2;
    depthDirNames += DEPTH_DIRNAME_1, DEPTH_DIRNAME_2;
    
    vector<string> sequencesNames;
    ReMedi::loadSequences(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), sequencesNames);
    
    vector< vector<string> > colorDirPaths, depthDirPaths;
    ReMedi::loadDirPaths(string(PARENT_PATH) + string(SEQUENCES_SUBDIR),
                 sequencesNames, KINECT_SUBSUBDIR, colorDirNames, colorDirPaths);
    ReMedi::loadDirPaths(string(PARENT_PATH) + string(SEQUENCES_SUBDIR),
                 sequencesNames, KINECT_SUBSUBDIR, depthDirNames, depthDirPaths);
    
    assert(colorDirPaths.size() == depthDirPaths.size());
    
    // *----------------------------*
    // | Read Kinect data sequences |
    // *----------------------------*
    
    KinectReader reader;
    
    // background sequence
    Sequence<ColorDepthFrame>::Ptr pBgSeq (new Sequence<ColorDepthFrame>(NUM_OF_VIEWS));
    reader.setPreallocation();
    reader.read(colorDirPaths[0], depthDirPaths[0], *pBgSeq);
    reader.setPreallocation(false);
    
    // and the rest of them (taking account the delays among views)
    vector<vector<int> > delays;
    ReMedi::loadDelaysFile(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), string(DELAYS_FILENAME), delays);
    
    vector<Sequence<ColorDepthFrame>::Ptr> sequences;
    for (int s = 1; s < colorDirPaths.size(); s++) // skip BG sequence
    {
        Sequence<ColorDepthFrame>::Ptr pSeq (new Sequence<ColorDepthFrame>(NUM_OF_VIEWS));
        reader.read(colorDirPaths[s], depthDirPaths[s], *pSeq);
        pSeq->setName(sequencesNames[s]);
        pSeq->setDelays(delays[s]);
        
        sequences.push_back(pSeq);
    }
    
    // *-----------------------------------*
    // | Create and parametrize the system |
    // *-----------------------------------*
    
    ReMedi sys;
    sys.setFramesResolution(X_RESOLUTION, Y_RESOLUTION);
    sys.setBackgroundSequence(pBgSeq);
    sys.setInputSequences(sequences);
    sys.setDefaultFrame(DEFAULT_FRAME);
    
    sys.setRegistererParameters(IR_NUM_OF_POINTS, IR_VIS_WND_HEIGHT, IR_VIS_WND_WIDTH, IR_VIS_VP, IR_VIS_HP, IR_VIS_DIST, IR_VIS_MARKER_RADIUS);
    sys.setTableModelerParameters(TM_LEAF_SIZE, TM_NORMAL_RADIUS, TM_SAC_ITERS, TM_SAC_DIST_THRESH, TM_TT_Y_OFFSET, TM_INTERACTIVE_BORDER_DIST, TM_CONF_LEVEL);
    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_K, BS_LRATE, BS_BGRATIO, BS_VARGEN); // some of these are re-set during the validation of the background subtractor

    // *------------------------------------*
    // | Validation of BackgroundSubtractor |
    // *------------------------------------*
    
    cout << "Validation of BackgroundSubtractor" << endl;
    
    // Define a set of parameters to validate
    
    vector<vector<double> > bsParameters;
    
    vector<double> modalities, learningRates, openingSizes;
    learningRates += 0;
    openingSizes += 0;
    
    vector<string> varNames;
    varNames += "modality";
    
    string bsValidationDir;
    
    modalities += COLOR, DEPTH, COLOR_WITH_SHADOWS, COLORDEPTH;
    vector<double> components, backgroundRatios, varGenThresholds;
    components += 2, 3, 4, 5, 6, 8, 10, 12, 15, 18, 20, 25, 30;
    backgroundRatios += 0.9, 0.99, 0.99999;
    varGenThresholds += 0.75, 1.5, 3, 6, 9, 12, 15, 18, 25;
    
    bsParameters += modalities, components, learningRates, backgroundRatios, varGenThresholds, openingSizes;
    
    varNames += "#mixture_components", "learning_rate", "background_ratio", "var_threshold_gen", "opening_size";
    bsValidationDir = "bs-mog2_results/";
//    modalities += ReMedi::COLOR, ReMedi::DEPTH, ReMedi::COLOR_WITH_SHADOWS, ReMedi::COLORDEPTH;
//    vector<double> maxFeatures, quantizationLevels, decisionThresholds;
//    maxFeatures += 2, 3, 4, 5, 6, 7, 8, 16, 32;
//    quantizationLevels += 1, 2, 3, 4, 5, 6, 7, 8, 16, 32;
//    decisionThresholds += 0.8, 0.9, 0.999, 0.99999;
//    
//    bsParameters += modalities, maxFeatures, learningRates, quantizationLevels, decisionThresholds, openingSizes;
//    
//    varNames += "#max_features", "learning_rate", "#quantization_levels", "var_decision_threshold", "opening_size";
//    bsValidationDir = "bs-gmg_results/";
    
    vector<string> foregroundMasksDirNames;
    foregroundMasksDirNames += FOREGROUND_MASKS_DIRNAME_1, FOREGROUND_MASKS_DIRNAME_2;
    
    vector< vector<string> > foregroundMasksDirPaths;
    ReMedi::loadDirPaths(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), sequencesNames, KINECT_SUBSUBDIR, foregroundMasksDirNames, foregroundMasksDirPaths);
    
    vector<Sequence<Frame>::Ptr> foregroundMasksSequences;
    for (int s = 1; s < foregroundMasksDirPaths.size(); s++) // skip BG sequence
    {
        Sequence<Frame>::Ptr pFgSeq (new Sequence<Frame>(NUM_OF_VIEWS));
        reader.read(foregroundMasksDirPaths[s], *pFgSeq);
        pFgSeq->setName(sequencesNames[s]);
        
        foregroundMasksSequences.push_back(pFgSeq);
    }
    
    cv::Mat bsCombinations;
    vector<cv::Mat> bsOverlaps;
//    validateBackgroundSubtraction(sys, sequences, bsParameters, foregroundMasksSequences, "bs-mog2_results/", "bs_validation.yml", bsCombinations, bsOverlaps);
    loadBackgroundSubtractionValidationFile("bs-mog2_results/bs_validation.yml", bsCombinations, bsOverlaps);
    
    cv::Mat bsMeans, bsStddevs;
    summarizeBackgroundSubtractionValidation(bsCombinations, bsOverlaps, bsMeans, bsStddevs);
    
    vector<vector<double> > filterParameters;
    filterParameters += modalities; // we do not want the best global result, but the best for each modality
    vector<int> filterIndices;
    filterIndices += 0; // modality is the 0-th parameter in "combinations"
    showValidationSummary(bsCombinations, filterParameters, filterIndices, bsMeans, bsStddevs);

    // *----------------------------------------*
    // | Validation of ObjectDetector (detection)  |
    // *----------------------------------------*
    
    cout << "Validation of ObjectDetector (detection)" << endl;
    
    vector<DetectionOutput> detectionGroundtruths;
    for (int s = 1; s < sequencesNames.size(); s++)
    {
        DetectionOutput groundtruth (string(PARENT_PATH) + string(OBJECTLABELS_SUBDIR), sequencesNames[s], "csv");
        groundtruth.setTolerance(0.15);
        detectionGroundtruths.push_back(groundtruth);
    }
    
    vector<vector<double> > mntrParameters;
    vector<double> morphLevels, leafSizes, clusterIntradists, minClusterSizes, detecionTolerances;
    morphLevels += -2, -1, 0, 1, 2;
    leafSizes += 0.005, 0.01;
    clusterIntradists += 2, 4, 6; // factor in function of leaf size
    minClusterSizes += 25, 50, 100;
    detecionTolerances += 0.05, 0.10, 0.15, 0.20;
    mntrParameters += morphLevels, leafSizes, clusterIntradists, minClusterSizes, detecionTolerances;
    
    cv::Mat bsBestCombinations;
    getBestCombinations(bsCombinations, filterParameters, filterIndices, 1, bsMeans, bsBestCombinations);
    
    cv::Mat mntrCombinations;
    vector<vector<vector<cv::Mat> > > mntrSgmtErrors;
    validateMonitorizationSegmentation(sys, sequences, bsBestCombinations, mntrParameters, detectionGroundtruths, "mntr_results/", "mntr_segmentation.yml", mntrCombinations, mntrSgmtErrors, false);
    loadMonitorizationSegmentationValidationFile("mntr_results/mntr_segmentation.yml", bsCombinations, mntrCombinations, mntrSgmtErrors);
    
    cv::Mat combinations, mntrSgmtMeans, mntrSgmtStddevs;
    summarizeMonitorizationSegmentationValidation(bsBestCombinations, mntrCombinations, mntrSgmtErrors, computeF1Score, combinations, mntrSgmtMeans, mntrSgmtStddevs);

    showValidationSummary(combinations, filterParameters, filterIndices, mntrSgmtMeans, mntrSgmtStddevs);
    
    // Combinations to test in the recognition
    cv::Mat sgmtBestCombinations;
    getBestCombinations(combinations, filterParameters, filterIndices, 1, mntrSgmtMeans, sgmtBestCombinations);
    
    // *------------------------*
    // | Recognition validation |
    // *------------------------*
    
    // Load objects models
    
    vector<int> objectsIDs;
    vector<string> objectsNames;
    vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > objectsViews;
    ReMedi::loadObjectsModels(string(PARENT_PATH) + string(OBJECTMODELS_SUBDIR), objectsIDs, objectsNames, objectsViews);
    
    vector<ObjectModel<pcl::PointXYZRGB>::Ptr> objectsModels (objectsIDs.size());
    for (int m = 0; m < objectsModels.size(); m++)
    {
        ObjectModel<pcl::PointXYZRGB>::Ptr pObjectModel ( new ObjectModel<pcl::PointXYZRGB>(objectsIDs[m], objectsNames[m], objectsViews[m]) );
        objectsModels[m] = pObjectModel;
    }
    
    sys.setObjectRecognizerParameters(objectsModels, DESCRIPTION_FPFH, RECOGNITION_MULTIOCULAR);
    sys.setObjectRecognizerPfhParameters(OR_PFHDESC_LEAFSIZE, OR_PFHDESC_MODEL_LEAFSIZE, OR_PFHDESC_NORMAL_RADIUS, OR_PFHDESC_MODEL_NORMAL_RADIUS, OR_PFHDESC_PFH_RADIUS, OR_PFHDESC_MODEL_PFH_RADIUS);
    
    vector<vector<double> > mntrRcgnParameters;
    vector<double> rcgnDescriptions;
    rcgnDescriptions += DESCRIPTION_FPFH, DESCRIPTION_PFHRGB;
    vector<double> rcgnStrategies;
    rcgnStrategies += RECOGNITION_MONOCULAR, RECOGNITION_MULTIOCULAR;
    mntrRcgnParameters += rcgnDescriptions, rcgnStrategies;
    
    
    vector<vector<vector<cv::Mat> > > mntrRcgnErrors;
    validateMonitorizationRecognition(sys, sequences, sgmtBestCombinations, mntrRcgnParameters, detectionGroundtruths, "mntr_results/", "mntr_recognition.yml", mntrRcgnErrors, false);
    
    return 0;
}

int main()
{
    return validation();
}
