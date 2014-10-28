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

// Auxiliary class to handle the scored detections kind of data
//
class ScoredDetections
{
public:
    ScoredDetections();
    ScoredDetections(std::vector<std::vector<int> > vids, std::vector<std::vector<pcl::PointXYZ> > positions, std::vector<std::vector< std::vector<float> > > scores);
    ScoredDetections(cv::Mat positions, cv::Mat scores);
    ScoredDetections(const ScoredDetections& rhs);
    ScoredDetections& operator=(const ScoredDetections& rhs);
    
    void set(std::vector<std::vector<int> > vids, std::vector<std::vector<pcl::PointXYZ> > positions, std::vector<std::vector< std::vector<float> > > scores);
    void get(std::vector<std::vector<int> >& vids, std::vector<std::vector<pcl::PointXYZ> >& positions, std::vector<std::vector< std::vector<float> > >& scores);
    std::vector<std::vector< int > > getVids();
    std::vector<std::vector< pcl::PointXYZ > > getPositions();
    std::vector<std::vector< std::vector<float> > > getScores();
    
    bool empty();

    void append(std::string filePath, std::string name);
    void load(std::string filePath, std::string name);
    
private:
    
    void setSparseRepresentation(cv::Mat positions, cv::Mat scores);
    void getSparseRepresentation(cv::Mat& positions, cv::Mat& scores);
    
    // instances-by-views
    std::vector<std::vector< int > >                    m_Vids;
    std::vector<std::vector< pcl::PointXYZ > >          m_Positions;
    // instances-by-views-by-models (float)
    std::vector<std::vector< std::vector<float> > >     m_Scores;
};

// ObjectDetector recognition performance
//void loadPrecomputedRecognitionScoresFile(std::string filePath, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& scores, vector<vector<vector<cv::Mat> > >& positions);
void loadMonitorizationRecognitionPrecomputedScoresFile(std::string filePath, int cid, int sid, std::vector<ScoredDetections>& scoredsFrames);

void loadMonitorizationRecognitionScoredDetections(std::string filePath, std::vector<int> combinationsIds, std::vector<int> sequencesIds, std::vector<std::vector<std::vector<ScoredDetections> > >& scoreds);

void _precomputeScores(ReMedi::Ptr pSys, vector<ColorDepthFrame::Ptr> frames, BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBS, cv::Mat combinations, int offset, std::string filePath, std::string id, ScoredDetections& scoreds);
void precomputeRecognitionScores(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> seqsIndices, cv::Mat combinations, std::vector<int> combsIndices, std::vector<DetectionOutput> detectionGroundtruths, string path, string filename, vector<vector<vector<ScoredDetections> > >& scoreds, int numOfThreads = NUM_OF_THREADS);

//void validateMonitorizationRecognition(ReMedi::Ptr pSys, std::vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat combinations, std::vector<std::vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

//void validateMonitorizationRecognition(vector<Sequence<ColorDepthFrame>::Ptr> sequences, vector<vector<vector<cv::Mat> > > positions, vector<vector<vector<cv::Mat> > > scores, vector<vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

void validateMonitorizationRecognition(ReMedi::Ptr pSys, std::vector<Sequence<ColorDepthFrame>::Ptr> sequences, std::vector<int> sequencesIndices, cv::Mat combinations, std::vector<int> combsIndices, std::vector<std::vector<std::vector<ScoredDetections> > > scoreds, std::vector<std::vector<double> > rcgnParameters, std::vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, std::vector<std::vector<std::vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);

void visualizeRecognitions(std::vector<ColorDepthFrame::Ptr> frames, std::vector<std::vector<std::vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > matches, std::vector<std::vector<std::vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > > rejections, float markersRadius, float lineWidth);



#endif /* defined(__remedi2__recognition_validation__) */
