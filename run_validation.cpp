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

// Declarations

void showValidationSummary(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, cv::Mat meanScores, cv::Mat sdScores, bool bMinimize = false);
void getBestCombinations(cv::Mat combinations, vector<vector<double> > parameters, vector<int> indices, int k, cv::Mat scores, cv::Mat& bestCombinations, bool bMinimize = false);
int validation();

// Implementations

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

    cv::Mat bsMeans, bsStddevs;
    summarizeBackgroundSubtractionValidation(bsCombinations, bsOverlaps, bsMeans, bsStddevs);

    vector<vector<double> > filterParameters;
    filterParameters += modalities; // we do not want the best global result, but the best for each modality
    vector<int> filterIndices;
    filterIndices += 0; // modality is the 0-th parameter in "combinations"
    showValidationSummary(bsCombinations, filterParameters, filterIndices, bsMeans, bsStddevs);

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
    
    cv::Mat bsBestCombinations;
    getBestCombinations(bsCombinations, filterParameters, filterIndices, 1, bsMeans, bsBestCombinations);
    
    cv::Mat sgmtCombinations;
    vector<vector<cv::Mat> > sgmtErrors;
//    validateMonitorizationSegmentation2(sys, sequences, bsBestCombinations, mntrParameters, detectionGroundtruths, "sgmt_results/", "sgmt_validation.yml", sgmtCombinations, sgmtErrors, false);
    loadMonitorizationSegmentationValidationFile2("Results/sgmt_results/sgmt_validation.yml", sgmtCombinations, sgmtErrors);

    cv::Mat sgmtMeans, sgmtStddevs;
    summarizeMonitorizationSegmentationValidation(sgmtCombinations, sgmtErrors, computeF1Score, sgmtMeans, sgmtStddevs);

    filterParameters += detecionTolerances;
    filterIndices += (bsParameters.size() + mntrParameters.size() - 1);
    showValidationSummary(sgmtCombinations, filterParameters, filterIndices, sgmtMeans, sgmtStddevs);
    
    // Focus on 15 cm
    filterParameters[1] = std::vector<double>(1, 0.15); // 0.15 m
    
    cv::Mat sgmtBestCombinations;
    getBestCombinations(sgmtCombinations, filterParameters, filterIndices, 1, sgmtMeans, sgmtBestCombinations);
    
    // *------------------------*
    // | Recognition validation |
    // *------------------------*
    
    // Load objects models
    
    std::string modelsPath = std::string(PARENT_PATH) + std::string(OBJECTMODELS_SUBDIR);

    std::vector<std::string> objectsNames;
    boost::split(objectsNames, OR_OBJECTS_NAMES, boost::is_any_of(","));
    
    vector<vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > objectsViews;
    ReMedi::loadObjectModels(modelsPath.c_str(), "PCDs/", objectsNames, objectsViews);
    
    vector<ObjectModel<pcl::PointXYZRGB>::Ptr> objectsModels (objectsNames.size());
    for (int m = 0; m < objectsModels.size(); m++)
    {
        ObjectModel<pcl::PointXYZRGB>::Ptr pObjectModel ( new ObjectModel<pcl::PointXYZRGB>(m, objectsNames[m], objectsViews[m]) );
        objectsModels[m] = pObjectModel;
    }
    
    pSys->setObjectRecognizerParameters(objectsModels, DESCRIPTION_PFHRGB, RECOGNITION_MULTIOCULAR);
    pSys->setObjectRecognizerPfhParameters(OR_PFHDESC_LEAFSIZE, OR_PFHDESC_MODEL_LEAFSIZE, OR_PFHDESC_NORMAL_RADIUS, OR_PFHDESC_MODEL_NORMAL_RADIUS, OR_PFHDESC_PFH_RADIUS, OR_PFHDESC_MODEL_PFH_RADIUS);
    
    vector<vector<vector<ScoredDetections> > > scoreds;
    precomputeRecognitionScores(pSys, sequences, sgmtBestCombinations, "Results/rcgn_results/", "rcgn_scores.yml", scoreds);
    
    vector<vector<double> > mntrRcgnParameters;
    vector<double> rcgnStrategies;
    rcgnStrategies += (double) RECOGNITION_MONOCULAR, (double) RECOGNITION_MULTIOCULAR;
    vector<double> rcgnTempCoherences;
    rcgnTempCoherences += 0, 1, 2, 3;
    mntrRcgnParameters += rcgnStrategies, rcgnTempCoherences;
    
//    vector<vector<vector<cv::Mat> > > mntrRcgnErrors;
//    validateMonitorizationRecognition(sys, sequences, sgmtBestCombinations, mntrRcgnParameters, detectionGroundtruths, "mntr_results/", "mntr_recognition.yml", mntrRcgnErrors, true);
    
    return 0;
}

int main()
{
    return validation();
}
