//
//  DetectionOutput.h
//  remedi
//
//  Created by Albert Clapés on 14/05/14.
//
//

#ifndef __remedi__DetectionOutput__
#define __remedi__DetectionOutput__

#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

using namespace std;

class DetectionOutput
{
public:
    DetectionOutput();
    DetectionOutput(int nviews, vector<int> nframes, int nobjects, float tol);
    DetectionOutput(vector< vector< vector< vector< pcl::PointXYZ > > > > positions, vector<int> delays);
    DetectionOutput(string path, string filename, string extension);
    DetectionOutput(const DetectionOutput& rhs);
    ~DetectionOutput();
    
    DetectionOutput& operator=(const DetectionOutput& rhs);
    
    void setPositions(vector< vector< vector< vector< pcl::PointXYZ > > > > positions, vector<int> delays);
    int getNumOfViews();
    vector<int> getNumOfFrames();
    int getNumOfObjects();
//    void setNumOfViews(int n);
//    void setNumOfFrames(int n);
//    void setNumOfObjects(int n);
    void setTolerance(float tol);
    
    void add(vector<vector<vector<pcl::PointXYZ> > > positions);
    void add(int view, int frame, int object, pcl::PointXYZ position);
    void remove(int view, int frame, int object, pcl::PointXYZ position);
    void remove(int view, int frame, int object, int i);
    void get(int view, int frame, int object, vector<pcl::PointXYZ>& positions);
    
    void set(vector<int> frames, int object, vector<vector<pcl::PointXYZ> > positions);
    
    void clear();
    void read(string path, string filename, string extension);
    void write(string path, string filename, string extension);
    
    int  getNumOfDetections();
    void getSegmentationResults(DetectionOutput groundtruth, int& tp, int& fn, int& fp);
    void getRecognitionResults(DetectionOutput groundtruth, int& tp, int& fn, int& fp);

    // in case this instance is a groundtruth instance
    // segmentation
    void getFrameSegmentationResults(vector<int> indices, vector<vector<pcl::PointXYZ> > predictions, cv::Mat& errors);
    void getFrameSegmentationResults(vector<int> indices, vector<vector<pcl::PointXYZ> > predictions, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > >& correspondences, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > >& rejections, cv::Mat& errors);
    
//    void getFrameSegmentationResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<pcl::PointXYZ> predictions, int& tp, int& fn, int& fp);
    void getFrameSegmentationResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<pcl::PointXYZ> predictions, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& correspondences, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& rejections, int& tp, int& fn, int& fp);
    
    // recognition
    void getFrameRecognitionResults(vector<int> indices, vector<vector<vector<pcl::PointXYZ> > > recognitions, cv::Mat& errors); // recognitions in each view, each kind of object, and each instance of object
    void getFrameRecognitionResults(vector<int> indices, vector<vector<vector<pcl::PointXYZ> > > recognitions, vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > >& correspondences, vector<vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > > >& rejections, cv::Mat& errors);
    
//    void getFrameRecognitionResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<vector<pcl::PointXYZ> > predictions, int& tp, int& fn, int& fp);
    void getFrameObjectRecognitionResults(vector<pcl::PointXYZ> groundtruth, vector<pcl::PointXYZ> recognitions, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& correspondences, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& rejections, int& tp, int& fn, int& fp);
    
private:
    float distance(pcl::PointXYZ p1, pcl::PointXYZ p2);
    
//    void getSegmentationFrameResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<vector<pcl::PointXYZ> > predictions, int& tp, int& fn, int& fp);
//    void getRecognitionFrameResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<vector<pcl::PointXYZ> > predictions, int& tp, int& fn, int& fp);

    // view, frame, model, model_instances_positions, (x,y,z) "real world" position
    vector< vector< vector< vector<pcl::PointXYZ> > > > m_Positions;
    vector< vector<bool> > m_Annotations; // wheter frames are annotated or not
    
    int m_NumOfViews;
    vector<int> m_NumOfFrames;
    int m_NumOfObjects;
    float m_Tol; // do not add a new detection if there is another very close
};

#endif /* defined(__remedi__DetectionOutput__) */
