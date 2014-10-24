//
//  segmentation_validation.h
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#ifndef __remedi2__segmentation_validation__
#define __remedi2__segmentation_validation__

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

// ObjectDetector segmentation validation and performance
//void loadMonitorizationSegmentationValidationFile(std::string filePath, cv::Mat& bsCombinations, cv::Mat& mntrCombinations, vector<vector<vector<cv::Mat> > >& errors);
void loadMonitorizationSegmentationValidationFile2(std::string filePath, cv::Mat& combinations, vector<vector<cv::Mat> >& errors);
//void validateMonitorizationSegmentation(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, cv::Mat& combinations, vector<vector<vector<cv::Mat> > >& errors, bool bQualitativeEvaluation = false);
void validateMonitorizationSegmentation2(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr> sequences, cv::Mat bsCombinations, vector<vector<double> > mntrParameters, vector<DetectionOutput> detectionGroundtruths, std::string path, std::string filename, cv::Mat& combinations, vector<vector<cv::Mat> >& errors, bool bQualitativeEvaluation = false);
void summarizeMonitorizationSegmentationValidation(cv::Mat combinations, vector<vector<cv::Mat> > errors, void (*f)(cv::Mat, cv::Mat, cv::Mat, cv::Mat&), cv::Mat& meanScores, cv::Mat& sdScores);

void visualizeSegmentations(vector<ColorDepthFrame::Ptr> frames, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > matches, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > rejections, float markersRadius, float lineWidth);

#endif /* defined(__remedi2__segmentation_validation__) */
