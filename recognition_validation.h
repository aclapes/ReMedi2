//
//  recognition_validation.h
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#ifndef __remedi2__recognition_validation__
#define __remedi2__recognition_validation__

#include <iostream>
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

class ScoredDetections
{
public:
    void toSparseRepresentation(cv::Mat& positions, cv::Mat& scores);
    
    std::vector<std::vector<int> > vids_;
    // instances-by-views
    std::vector<std::vector<pcl::PointXYZ> > positions_;
    // instances-by-views-by-models (float)
    std::vector<std::vector< std::vector<float> > > scores_;
};

// ObjectDetector recognition performance
void loadPrecomputedRecognitionScoresFile(std::string filePath, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& scores, vector<vector<vector<cv::Mat> > >& positions);

void _precomputeScores(ReMedi::Ptr pSys, vector<ColorDepthFrame::Ptr> frames, BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS, cv::Mat combinations, int offset, std::string filePath, std::string id, ScoredDetections& scoreds);
void precomputeRecognitionScores(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsIndices, cv::Mat combinations, std::vector<int> combsIndices, string path, string filename, vector<vector<vector<ScoredDetections> > >& scoreds);

//void validateMonitorizationRecognition(ReMedi::Ptr pSys, std::vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat combinations, std::vector<std::vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

void validateMonitorizationRecognition(vector<Sequence<ColorDepthFrame>::Ptr> sequences, vector<vector<vector<cv::Mat> > > positions, vector<vector<vector<cv::Mat> > > scores, vector<vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

void visualizeRecognitions(std::vector<ColorDepthFrame::Ptr> frames, std::vector<std::vector<std::vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, std::vector<std::vector<std::vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > rejections, float markersRadius, float lineWidth);



#endif /* defined(__remedi2__recognition_validation__) */
