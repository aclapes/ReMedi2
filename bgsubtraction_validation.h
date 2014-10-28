//
//  bgsubtraction_validation.h
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#ifndef __remedi2__bgsubtraction_validation__
#define __remedi2__bgsubtraction_validation__

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

// Background subtraction validation and performance
int loadBackgroundSubtractionValidationFile(string filePath, cv::Mat& combinations, vector<cv::Mat>& overlaps);
void validateBackgroundSubtraction(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr > sequences, vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename, cv::Mat& combinations, std::vector<cv::Mat>& overlaps);
void summarizeBackgroundSubtractionValidation(vector<Sequence<Frame>::Ptr> foregroundMasksSequences, std::vector<int> subjectsIds, cv::Mat combinations, vector<cv::Mat> overlaps, std::vector<cv::Mat>& summaries); // summary for each view

#endif /* defined(__remedi2__bgsubtraction_validation__) */
