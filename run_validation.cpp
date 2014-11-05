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

#include "bgsubtraction_validation.h"
#include "segmentation_validation.h"
#include "recognition_validation.h"

#include <pcl/console/parse.h>

// Global variables

int g_NumOfThreads = NUM_OF_THREADS;

// Declarations

void showValidationSummary(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat meanScores, cv::Mat sdScores, bool bMinimize = false);
void getBestCombinations(cv::Mat combinations, std::vector<cv::Mat> performances, vector<vector<double> > parameters, vector<int> indices, int k, cv::Mat& bestCombinations, bool bMinimize = false);
int validation();

// Implementations

void f(vector<cv::Mat> performances, std::vector<int> subjectsIds, cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat& means, cv::Mat& stddevs)
{
    cv::Mat fcombinations;
    expandParameters<double>(parameters, fcombinations);
    
    means.create(fcombinations.rows, NUM_OF_VIEWS, cv::DataType<float>::type);
    stddevs.create(fcombinations.rows, NUM_OF_VIEWS, cv::DataType<float>::type);
    
    for (int v = 0; v < NUM_OF_VIEWS; v++)
    {
        // Performances at sequence level
        cv::Mat perfsViewSeqsSmry = performances[v];
        
        // Summarize even more: sequence level to subject level
        cv::Mat perfsViewSbjsSmry (perfsViewSeqsSmry.rows, NUM_OF_SUBJECTS, perfsViewSeqsSmry.type());
        int sbjstart = 0;
        int sbjId = 0;
        for (int i = 0; i < perfsViewSeqsSmry.cols; i++) // iterate the sequences
        {
            if (sbjId != subjectsIds[i])
            {
                cv::Mat roi (perfsViewSeqsSmry, cv::Rect(sbjstart, 0, (i - sbjstart), perfsViewSeqsSmry.rows));
                cv::reduce(roi, perfsViewSbjsSmry.col(sbjId), 1, CV_REDUCE_AVG); // average sequences of same subject
                
                sbjstart = i;
                sbjId++;
            }
        }
		cv::Mat roi (perfsViewSeqsSmry, cv::Rect(sbjstart, 0, ((perfsViewSeqsSmry.cols - 1) - sbjstart), perfsViewSeqsSmry.rows));
        cv::reduce(roi, perfsViewSbjsSmry.col(sbjId), 1, CV_REDUCE_AVG); // average sequences of same subject

        // The rest of the work: to use all but one subject to estimate the best combination,
        // and finally test it in that one subject
        
        for (int c = 0; c < fcombinations.rows; c++)
        {
            cv::Mat mask (combinations.rows, 1, cv::DataType<uchar>::type, cv::Scalar(255));
            cv::Mat aux;
            for (int p = 0; p < fcombinations.cols; p++)
            {
                aux = ( combinations.col(indices[p]) == fcombinations.at<double>(c,p) );
                mask &= aux;
            }
            
            cv::Mat perfsSubjects (NUM_OF_SUBJECTS, 1, cv::DataType<float>::type);
            
            for (int i = 0; i < NUM_OF_SUBJECTS; i++)
            {
                // Tricky piece of code: the mean of combinations' perfs except the subject ones,
                // is got by summing all of them, minus the subject ones, divided by the cardinality
                // of subjects-1
                cv::Mat sum;
                cv::reduce(perfsViewSbjsSmry, sum, 1, CV_REDUCE_AVG);

				cv::Mat avgExceptSbj = (sum - perfsViewSbjsSmry.col(i) / (NUM_OF_SUBJECTS - 1));

                double minVal, maxVal;
                cv::Point minIdx, maxIdx;
                cv::minMaxLoc(avgExceptSbj, &minVal, &maxVal, &minIdx, &maxIdx, mask);
                
                perfsSubjects.at<float>(i,0) = perfsViewSbjsSmry.at<float>(maxIdx.y,i);
            }
            
            cv::Scalar mean, stddev;
            cv::meanStdDev(perfsSubjects, mean, stddev);
            means.at<float>(c,v) = mean.val[0];
            stddevs.at<float>(c,v) = stddev.val[0];
        }
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
        cv::Mat aux;
        for (int p = 0; p < fcombinations.cols; p++)
        {
            aux = ( combinations.col(indices[p]) == fcombinations.at<double>(c,p) );
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

void getBestCombinations(cv::Mat combinations, std::vector<cv::Mat> performances, vector<vector<double> > parameters, vector<int> indices, int k, cv::Mat& bestCombinations, bool bMinimize)
{
    cv::Mat performancesViews (performances[0].rows, performances.size(), cv::DataType<float>::type);
    for (int v = 0; v < performances.size(); v++)
        cv::reduce(performances[v], performancesViews.col(v), 1, CV_REDUCE_AVG);
    
    cv::Mat performancesHMean;
    cvx::harmonicMean(performancesViews, performancesHMean, 1);
    
    // DEBUG: print
    // ------------------------------------------
//    cv::Mat aux;
//    cv::Mat combinationsTmp;
//    combinations.convertTo(combinationsTmp, performancesHMean.type());
//    cv::hconcat(combinationsTmp, performancesHMean, aux);
//    std::cout << aux << std::endl;
    // ------------------------------------------
    
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
        cvx::indexMat(performancesHMean, indexedMeans, mask);
        
        cv::Mat I;
        cv::sortIdx( indexedMeans, I, (bMinimize ? CV_SORT_ASCENDING : CV_SORT_DESCENDING) | CV_SORT_EVERY_COLUMN );
        
        for (int i = 0; i < k; i++)
            indexedCombinations.row(I.at<int>(i,0)).copyTo( bestCombinations.row(c * k + i) );
    }
}

int validation(std::string rcgnScoresFilename, std::vector<int> rcgnModalitiesIndices, std::vector<int> seqsIndices)
{
    vector<string> colorDirNames, depthDirNames;
    colorDirNames += COLOR_DIRNAME_1, COLOR_DIRNAME_2;
    depthDirNames += DEPTH_DIRNAME_1, DEPTH_DIRNAME_2;
    
    std::vector<std::string> sequencesNames;
    std::vector<int> sequencesSids;
    ReMedi::loadSequences(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), sequencesNames, sequencesSids);
    
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
    
    ReMedi::Ptr pSys (new ReMedi);
    pSys->setFramesResolution(X_RESOLUTION, Y_RESOLUTION);
    pSys->setBackgroundSequence(pBgSeq);
    pSys->setInputSequences(sequences);
    pSys->setDefaultFrame(DEFAULT_FRAME);
    
    pSys->setRegistererParameters(IR_NUM_OF_POINTS, IR_VIS_WND_HEIGHT, IR_VIS_WND_WIDTH, IR_VIS_VP, IR_VIS_HP, IR_VIS_DIST, IR_VIS_MARKER_RADIUS);
    pSys->setTableModelerParameters(TM_LEAF_SIZE, TM_NORMAL_RADIUS, TM_SAC_ITERS, TM_SAC_DIST_THRESH, TM_TT_Y_OFFSET, TM_INTERACTIVE_BORDER_DIST, TM_CONF_LEVEL);
    pSys->setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_K, BS_LRATE, BS_BGRATIO, BS_VARGEN, BS_MORPHOLOGY); // some of these are re-set during the validation of the background subtractor

    // *------------------------------------*
    // | Validation of BackgroundSubtractor |
    // *------------------------------------*
    
    cout << "Validation of BackgroundSubtractor" << endl;
    
    // Define a set of parameters to validate
    
    vector<vector<double> > bsParameters;
    
    vector<string> varNames;
    varNames += "modality";
    
    vector<double> modalities, components, learningRates, backgroundRatios, varGenThresholds, openingSizes;
    modalities += COLOR, DEPTH, COLOR_WITH_SHADOWS, COLORDEPTH;
    components += 2, 3, 5, 10, 20, 30;
    learningRates += 0;
    backgroundRatios += 0.9, 0.99, 0.99999;
    varGenThresholds += 9, 16, 25, 36;
    openingSizes += -2, -1, 0, 1, 2;
    
    bsParameters += modalities, components, learningRates, backgroundRatios, varGenThresholds, openingSizes;
    
    varNames += "#mixture_components", "learning_rate", "background_ratio", "var_threshold_gen", "opening_size";

//    modalities += ReMedi::COLOR, ReMedi::DEPTH, ReMedi::COLOR_WITH_SHADOWS, ReMedi::COLORDEPTH;
//    vector<double> maxFeatures, quantizationLevels, decisionThresholds;
//    maxFeatures += 2, 3, 4, 5, 6, 7, 8, 16, 32;
//    quantizationLevels += 1, 2, 3, 4, 5, 6, 7, 8, 16, 32;
//    decisionThresholds += 0.8, 0.9, 0.999, 0.99999;
//    
//    bsParameters += modalities, maxFeatures, learningRates, quantizationLevels, decisionThresholds, openingSizes;
//    
//    varNames += "#max_features", "learning_rate", "#quantization_levels", "var_decision_threshold", "opening_size";
    
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
    loadBackgroundSubtractionValidationFile("Results/bs-mog2_results/bs_validation.yml", bsCombinations, bsOverlaps);
    
    vector<vector<double> > filterParameters;
    filterParameters += modalities; // we do not want the best global result, but the best for each modality
    vector<int> filterIndices;
    filterIndices += 0; // modality is the 0-th parameter in "combinations"
    
    std::vector<cv::Mat> bsPerformanceSummaries;
    summarizeBackgroundSubtractionValidation(foregroundMasksSequences, sequencesSids, bsCombinations, bsOverlaps, bsPerformanceSummaries);
    
    cv::Mat bsMeans, bsStddevs;
    f(bsPerformanceSummaries, sequencesSids, bsCombinations, filterParameters, filterIndices, bsMeans, bsStddevs);
    
    std::cout << bsMeans << std::endl;
    std::cout << bsStddevs << std::endl;
    
    cv::Mat bsBestCombinations;
    getBestCombinations(bsCombinations, bsPerformanceSummaries, filterParameters, filterIndices, 1, bsBestCombinations);
    
    std::cout << bsBestCombinations << std::endl;

    // *---------------------------------------------*
    // | Validation of ObjectDetector (segmentation) |
    // *---------------------------------------------*
    
    cout << "Validation of ObjectDetector (segmentation)" << endl;
    
    vector<DetectionOutput> detectionGroundtruths;
    for (int s = 1; s < sequencesNames.size(); s++)
    {
        DetectionOutput groundtruth (string(PARENT_PATH) + string(OBJECTLABELS_SUBDIR), sequencesNames[s], "csv");
        groundtruth.setTolerance(0.15);
        detectionGroundtruths.push_back(groundtruth);
    }
    
    vector<vector<double> > mntrParameters;
    vector<double> leafSizes, clusterIntradists, minClusterSizes, detecionTolerances;
    leafSizes += 0.005, 0.01;
    clusterIntradists += 2, 4, 6; // factor in function of leaf size
    minClusterSizes += 25, 50, 100, 150;
    detecionTolerances += 0.05, 0.10, 0.15, 0.20;
    mntrParameters += leafSizes, clusterIntradists, minClusterSizes, detecionTolerances;
    
    cv::Mat sgmtCombinations;
    vector<vector<cv::Mat> > sgmtErrors;
//    validateMonitorizationSegmentation2(pSys, sequences, seqsIndices, bsBestCombinations, mntrParameters, detectionGroundtruths, "Results/sgmt_results/", "sgmt_validation.yml", sgmtCombinations, sgmtErrors, false);
    loadMonitorizationSegmentationValidationFile2("Results/sgmt_results/sgmt_validation.yml", sgmtCombinations, sgmtErrors);

    std::vector<cv::Mat> sgmtPerformanceSummaries, sgmtErrorsSummaries; // F-scores, errors (TP,FN,FP)
    summarizeMonitorizationSegmentationValidation2(sequences, sgmtCombinations, sgmtErrors, computeF1Score, sgmtPerformanceSummaries, sgmtErrorsSummaries);

    filterParameters += detecionTolerances;
    filterIndices += (bsParameters.size() + mntrParameters.size() - 1);

//    showValidationSummary(sgmtCombinations, filterParameters, filterIndices, sgmtMeans, sgmtStddevs);
    cv::Mat sgmtMeans, sgmtStddevs;
    f(sgmtPerformanceSummaries, sequencesSids, sgmtCombinations, filterParameters, filterIndices, sgmtMeans, sgmtStddevs);
    
    std::cout << sgmtMeans << std::endl;
    std::cout << sgmtStddevs << std::endl;

    cv::Mat sgmtBestCombinations;
    getBestCombinations(sgmtCombinations, sgmtPerformanceSummaries, filterParameters, filterIndices, 1, sgmtBestCombinations);
    
    // *------------------------*
    // | Recognition validation |
    // *------------------------*
    
    std::cout << "Validation of ObjectRecognizer (object recognition)" << std::endl;

    // Load objects models
    
    std::string modelsPath = std::string(PARENT_PATH) + std::string(OBJECTMODELS_SUBDIR);

    std::vector<std::string> objectsNamesStr;
    boost::split(objectsNamesStr, OR_OBJECTS_NAMES, boost::is_any_of(","));
    
    vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > objectsViews;
    ReMedi::loadObjectModels(modelsPath.c_str(), "PCDs/", objectsNamesStr, objectsViews);
    
    vector<ObjectModel<pcl::PointXYZRGB>::Ptr> objectsModels (objectsNamesStr.size());
    for (int m = 0; m < objectsModels.size(); m++)
    {
        ObjectModel<pcl::PointXYZRGB>::Ptr pObjectModel ( new ObjectModel<pcl::PointXYZRGB>(m, objectsNamesStr[m], objectsViews[m]) );
        objectsModels[m] = pObjectModel;
    }
    
    std::vector<std::string> objectsRejectionsStr;
    boost::split(objectsRejectionsStr, OR_OBJECTS_REJECTIONS, boost::is_any_of(","));
    
    std::vector<float> objectsRejections;
    std::vector<std::string>::iterator it;
    for (it = objectsRejectionsStr.begin(); it != objectsRejectionsStr.end(); it++)
        objectsRejections.push_back( stof(*it) );
    
    pSys->setObjectRecognizerParameters(objectsModels, objectsRejections, DESCRIPTION_PFHRGB, RECOGNITION_MULTIOCULAR);
    pSys->setObjectRecognizerPfhParameters(OR_PFHDESC_LEAFSIZE, OR_PFHDESC_MODEL_LEAFSIZE, OR_PFHDESC_NORMAL_RADIUS, OR_PFHDESC_MODEL_NORMAL_RADIUS, OR_PFHDESC_PFH_RADIUS, OR_PFHDESC_MODEL_PFH_RADIUS, OR_POINT_REJECTION_THRESH);

    filterParameters[1] = std::vector<double>(1, 0.15); // 0.15 m
    f(sgmtPerformanceSummaries, sequencesSids, sgmtCombinations, filterParameters, filterIndices, sgmtMeans, sgmtStddevs);
    
    std::cout << sgmtMeans << std::endl;
    std::cout << sgmtStddevs << std::endl;
    
    getBestCombinations(sgmtCombinations, sgmtPerformanceSummaries, filterParameters, filterIndices, 1, sgmtBestCombinations);
    
    std::cout << sgmtBestCombinations << std::endl;
    
    std::vector<std::vector<std::vector<ScoredDetections> > > scoreds;
    precomputeRecognitionScores(pSys, sequences, seqsIndices, sgmtBestCombinations, rcgnModalitiesIndices, detectionGroundtruths, "Results/rcgn_results/", rcgnScoresFilename, scoreds, g_NumOfThreads); // TODO: remove detectionGroundtruths from function call
    
//    loadMonitorizationRecognitionScoredDetections("Results/rcgn_results/rcgn_scores.yml", rcgnModalitiesIndices, seqsIndices, scoreds);
//
//    vector<vector<double> > mntrRcgnParameters;
//    vector<double> rcgnStrategies;
//    rcgnStrategies += (double) RECOGNITION_MONOCULAR, (double) RECOGNITION_MULTIOCULAR;
//    vector<double> rcgnConsensus;
//    rcgnStrategies += (double) RECOGNITION_INTERVIEW_AVG, (double) RECOGNITION_INTERVIEW_DISTWEIGHTED;
//    vector<double> rcgnTempCoherences;
//    rcgnTempCoherences += 0, 1, 2, 3;
//    mntrRcgnParameters += rcgnStrategies, rcgnTempCoherences;
//    
//    std::vector<std::vector<cv::Mat> > mntrRcgnErrors;
//    validateMonitorizationRecognition(pSys, sequences, seqsIndices, sgmtBestCombinations.row(2),  scoreds, mntrRcgnParameters, detectionGroundtruths, "Results/rcgn_results/", "rcgn_validation.yml", mntrRcgnErrors, true);
    
    return 0;
}

int main(int argc, char** argv)
{
    if (pcl::console::find_argument(argc, argv, "-T") > 0)
        pcl::console::parse(argc, argv, "-T", g_NumOfThreads); // global variable modified
    
    std::vector<int> rcgnModalitiesIndices;
    if (pcl::console::find_argument(argc, argv, "-Rm") > 0)
    {
        std::string rcgnModalitiesIndicesStr;
        pcl::console::parse(argc, argv, "-Rm", rcgnModalitiesIndicesStr);
    
        std::vector<std::string> rcgnModalitiesIndicesStrL;
        boost::split(rcgnModalitiesIndicesStrL, rcgnModalitiesIndicesStr, boost::is_any_of(","));

        for (std::vector<std::string>::iterator it = rcgnModalitiesIndicesStrL.begin(); it != rcgnModalitiesIndicesStrL.end(); it++)
            rcgnModalitiesIndices.push_back( stoi(*it) );
    }
    
    std::vector<int> sequencesIndices;
    if (pcl::console::find_argument(argc, argv, "-Rs") > 0)
    {
        std::string sequencesIndicesStr;
        pcl::console::parse(argc, argv, "-Rs", sequencesIndicesStr);

        std::vector<std::string> rcgnModalitiesIndicesStrL, sequencesIndicesStrL;
        boost::split(sequencesIndicesStrL, sequencesIndicesStr, boost::is_any_of(","));
        
        for (std::vector<std::string>::iterator it = sequencesIndicesStrL.begin(); it != sequencesIndicesStrL.end(); it++)
            sequencesIndices.push_back( stoi(*it) );
    }
    
    std::string rcgnScoresFilename = DEFAULT_RCGN_SCORES_FILENAME;
    if (pcl::console::find_argument(argc, argv, "-O") > 0)
    {
        pcl::console::parse(argc, argv, "-O", rcgnScoresFilename);
    }
    
    return validation(rcgnScoresFilename, rcgnModalitiesIndices, sequencesIndices);
}
