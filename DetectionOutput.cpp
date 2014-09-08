//
//  DetectionOutput.cpp
//  remedi
//
//  Created by Albert Clapés on 14/05/14.
//
//

#include "DetectionOutput.h"

#include <math.h>
#include <assert.h>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include "cvxtended.h"

DetectionOutput::DetectionOutput()
: m_NumOfViews(0), m_NumOfObjects(0), m_Tol(0.07)
{
    
}

DetectionOutput::DetectionOutput(int nviews, vector<int> nframes, int nobjects, float tol)
: m_NumOfViews(nviews), m_NumOfFrames(nframes), m_NumOfObjects(nobjects), m_Tol(tol)
{
    m_Positions.resize(m_NumOfViews);
    m_Annotations.resize(m_NumOfViews);
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        m_Positions[v].resize(m_NumOfFrames[v]);
        for (int f = 0; f < m_NumOfFrames[v]; f++)
            m_Positions[v][f].resize(m_NumOfObjects);
        
        m_Annotations[v].resize(m_NumOfFrames[v], false);
    }
}

DetectionOutput::DetectionOutput(vector< vector< vector< vector< pcl::PointXYZ > > > > positions, vector<int> delays)
: m_Tol(0.07)
{
    setPositions(positions, delays);
}

DetectionOutput::DetectionOutput(string path, string filename, string extension)
: m_NumOfObjects(0), m_Tol(0.07)
{
    read(path, filename, extension);
}

DetectionOutput::DetectionOutput(const DetectionOutput& rhs)
{
    *this = rhs;
}

DetectionOutput::~DetectionOutput()
{
}

DetectionOutput& DetectionOutput::operator=(const DetectionOutput& rhs)
{
    if (this != &rhs)
    {
        m_Positions = rhs.m_Positions;

        if (m_Positions.size() > 0)
        {
            m_NumOfViews = m_Positions.size();
            for (int i = 0; i < m_NumOfViews; i++)
                m_NumOfFrames.push_back(m_Positions[i].size());
            m_NumOfObjects = m_Positions[0][0].size();
        }
        m_Annotations = rhs.m_Annotations;
        m_Tol = rhs.m_Tol;
    }
    
    return *this;
}

void DetectionOutput::setPositions(vector<vector<vector<vector<pcl::PointXYZ > > > > positions, vector<int> delays)
{
    m_Positions = positions;
    
    // Assertions
    for (int v = 0; v < m_Positions.size(); v++)
    {
        // assert num of frames
        assert( m_Positions[0].size() == m_Positions[v].size() );
        for (int f = 0; f < m_Positions[v][0].size(); f++)
            assert( m_Positions[0][0].size() == m_Positions[v][f].size() );
    }
    
    // Set important variables
    m_NumOfViews = m_Positions.size();
    m_NumOfFrames.resize(m_NumOfViews);
    m_Annotations.resize(m_NumOfViews);
    for (int v = 0; v < m_NumOfViews; v++)
    {
        m_NumOfFrames[v] = m_Positions[v].size();
        m_Annotations[v].resize(m_Positions[v].size(), true);
    }
    m_NumOfObjects = m_Positions[0][0].size(); // asserted
}

int DetectionOutput::getNumOfViews()
{
    return m_NumOfViews;
}

vector<int> DetectionOutput::getNumOfFrames()
{
    return m_NumOfFrames;
}

int DetectionOutput::getNumOfObjects()
{
    return m_NumOfObjects;
}

//void DetectionOutput::setNumOfViews(int n)
//{
//    m_NumOfViews = n;
//}
//
//void DetectionOutput::setNumOfFrames(int n)
//{
//    m_NumOfFrames = n;
//}
//
//void DetectionOutput::setNumOfObjects(int n)
//{
//    m_NumOfObjects = n;
//}

void DetectionOutput::setTolerance(float tol)
{
    m_Tol = tol;
}

float DetectionOutput::distance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) + powf(p1.z - p2.z, 2));
}

void DetectionOutput::add(vector<vector<vector<pcl::PointXYZ> > > positions)
{
    if (m_Positions.size() != positions.size())
    {
        m_Positions.resize(positions.size());
        m_Annotations.resize(positions.size());
    }
    
    for (int v = 0; v < positions.size(); v++)
    {
        m_Positions[v].push_back(positions[v]);
        m_Annotations[v].resize(positions[v].size(), true);
    }
}

void DetectionOutput::add(int view, int frame, int object, pcl::PointXYZ position)
{
    bool collision = false;
    for (int i = 0; i < m_Positions[view][frame][object].size() && !collision; i++)
        collision = distance(m_Positions[view][frame][object][i], position) < m_Tol;
        
    if (collision)
        m_Positions[view][frame][object].push_back(position);
    
    m_Annotations[view][frame] = true;
}

void DetectionOutput::remove(int view, int frame, int object, pcl::PointXYZ position)
{
    bool collision = false;
    int idx;
    for (int i = 0; i < m_Positions[view][frame][object].size() && !collision; i++)
    {
        collision = distance(m_Positions[view][frame][object][i], position) < m_Tol;
        if (collision) idx = i;
    }
    
    if (collision)
    {
        m_Positions[view][frame][object].erase(m_Positions[view][frame][object].begin() + idx);
        
        if (m_Positions[view][frame][object].size() == 0)
            m_Annotations[view][frame] = false;
    }
}

void DetectionOutput::remove(int view, int frame, int object, int i)
{
    m_Positions[view][frame][object].erase(m_Positions[view][frame][object].begin() + i);
    
    if (m_Positions[view][frame][object].size() == 0)
        m_Annotations[view][frame] = false;
}

void DetectionOutput::get(int view, int frame, int object, vector<pcl::PointXYZ>& positions)
{
    positions = m_Positions[view][frame][object];
}

void DetectionOutput::set(vector<int> frames, int object, vector<vector<pcl::PointXYZ> > positions)
{
    assert(frames.size() == positions.size());
    
    for (int v = 0; v < frames.size(); v++)
    {
        for (int i = 0; i < positions[v].size(); i++)
            m_Positions[v][frames[v]][object].push_back(positions[v][i]);
        m_Annotations[v][frames[v]] = true;
    }
}

void DetectionOutput::clear()
{
    m_Positions.clear();
    m_Annotations.clear();
    
    m_NumOfViews = 0;
    m_NumOfFrames.clear();
    m_NumOfObjects = 0;
}

void DetectionOutput::write(string path, string filename, string extension)
{
    // Draw set points
    for (int v = 0; v < m_NumOfViews; v++)
    {
        ofstream outFile;
        string outputPath = path + filename + "_" + to_string(v) + "." + extension;
        outFile.open(outputPath, ios::out);
        
        // create two view files
        for (int f = 0; f < m_NumOfFrames[v]; f++)
        {
            for (int o = 0; o < m_NumOfObjects; o++)
            {
                outFile << o << ":";
                
                vector< pcl::PointXYZ > tmp = m_Positions[v][f][o];
                
                for (int p = 0; p < tmp.size(); p++)
                {
                    outFile << tmp[p].x << "," << tmp[p].y << "," << tmp[p].z << ";";
                    
                } outFile << "\t";
            } outFile << endl;
        }
        outFile.close();
    }
    
    cout << "DetectionOutput written successfully to " << path << "." << endl;
}

//void DetectionOutput::read(string path, string filename, string extension)
//{
//    m_Positions.resize(m_NumOfViews);
//    
//    for (int i = 0; i < m_NumOfViews; i++)
//    {
//        m_Positions[i].resize(m_NumOfFrames[i]);
//        for (int j = 0; j < m_NumOfFrames[i]; j++)
//        {
//            m_Positions[i][j].resize(m_NumOfObjects);
//        }
//    }
//    
//    for (int v = 0; v < m_NumOfViews; v++)
//    {
//        ifstream inFile;
//        inFile.open(path + filename + "_" + to_string(v) + "." + extension, ios::in);
//
//        string line;
//        int f = 0; // number of lines, i.e. [f]rames
//        while( getline(inFile, line) )
//        {
//            vector<string> objects_sublines; // ex: 1:202,22;104,123;
//            boost::split(objects_sublines, line, boost::is_any_of("\t"));
//            
//            for (int o = 0; o < objects_sublines.size() - 1; o++)
//            {
//                vector<string> object_struct;
//                boost::split(object_struct, objects_sublines[o], boost::is_any_of(":"));
//                
//                int oid = stoi(object_struct[0]); // object id
//                if (object_struct[1].size() <= 2)
//                    continue;
//                
//                vector<string> positions;
//                boost::split(positions, object_struct[1], boost::is_any_of(";")); // object id's positions
//                
//                for (int p = 0; p < positions.size() - 1; p++)
//                {
//                    vector<string> coordinates;
//                    boost::split(coordinates, positions[p], boost::is_any_of(","));
//                    
//                    string::size_type sz;
//                    float x = stof(coordinates[0], &sz);
//                    float y = stof(coordinates[1], &sz);
//                    float z = stof(coordinates[2], &sz);
//                    m_Positions[v][f][oid].push_back( pcl::PointXYZ(x,y,z) );
//                }
//            }
//            f++;
//        }
//        
//        inFile.close();
//    }
//    
//    cout << "DetectionOutput read successfully from " << path << "." << endl;
//}

void DetectionOutput::read(string path, string filename, string extension)
{
    clear();
    
    ifstream inFile;
    string inFilePath = path + filename + "_" + to_string(m_NumOfViews) + "." + extension; // num of views is already set to 0 in clear() calling
    inFile.open(inFilePath, ios::in);
    
    while (inFile.is_open())
    {
        // First line in file is the delay in that view
        string line;
        getline(inFile, line);
        int delay = stoi(line);
        
        vector< vector< vector<pcl::PointXYZ> > > positionsView; // all positions in a view
        
        int f = 0; // number of lines, i.e. [f]rames
        while( getline(inFile, line) )
        {
            vector< vector<pcl::PointXYZ> > positionsViewFrame; // all positions in view's frame
            
            // parsing stuff reads objects' positions in a file line representing
            // the objects' appearences in a frame, in this format:
            //      idobject:x_{idobject,idinstance}
            //  ex:
            // 0:x_{0,0},y_{0,0},z_{0,0};x_{0,1},y_{0,1},z_{0,1};\t1:x_{1,0},y_{1,0},z_{1,0};x_{1,1},y_{1,1},z_{1,1};\t...
            //
            vector<string> objects_sublines;
            boost::split(objects_sublines, line, boost::is_any_of("\t"));
            
            // If not done, set the number of objects
            if (m_NumOfObjects == 0)
                m_NumOfObjects = objects_sublines.size() - 1;
            else // Must be consistent thorughtout views and frames
                assert ((objects_sublines.size() - 1) == m_NumOfObjects);
            
            positionsViewFrame.resize(m_NumOfObjects); // store that read positions in the proper structures
            for (int o = 0; o < m_NumOfObjects; o++)
            {
                vector<string> object_struct;
                boost::split(object_struct, objects_sublines[o], boost::is_any_of(":"));
                
                int oid = stoi(object_struct[0]); // object id
                if (object_struct[1].size() <= 2)
                    continue;
                
                vector<string> positions;
                boost::split(positions, object_struct[1], boost::is_any_of(";")); // object id's positions
                
                for (int p = 0; p < positions.size() - 1; p++)
                {
                    vector<string> coordinates;
                    boost::split(coordinates, positions[p], boost::is_any_of(","));
                    
                    string::size_type sz;
                    float x = stof(coordinates[0], &sz);
                    float y = stof(coordinates[1], &sz);
                    float z = stof(coordinates[2], &sz);
                    positionsViewFrame[oid].push_back( pcl::PointXYZ(x,y,z) );
                }
            }
            positionsView.push_back(positionsViewFrame);
            
            f++;
        }
        m_NumOfFrames.push_back(f);
        
        m_Positions.push_back(positionsView);
        m_Annotations.push_back(vector<bool>(positionsView.size(), true));
        
        inFile.close();
        m_NumOfViews++;

        inFilePath = path + filename + "_" + to_string(m_NumOfViews++) + "." + extension;
        inFile.open(inFilePath, ios::in);
    }
}

int DetectionOutput::getNumOfDetections()
{
    int count = 0;
    
    for (int v = 0; v < m_NumOfViews; v++)
        for (int f = 0; f < m_Positions[v].size(); f++)
            for (int o = 0; o < m_NumOfObjects; o++)
                count += m_Positions[v][f][o].size();
    
    return count;
}

void DetectionOutput::getSegmentationResults(DetectionOutput groundtruth, int& tp, int& fn, int& fp)
{
    assert( m_NumOfViews == m_Positions.size() );
    int gtsize = groundtruth.m_Positions.size();
    assert( m_NumOfViews == gtsize );
    
    tp = fn = fp = 0;
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        for (int f = 0; f < groundtruth.m_Positions[v].size() && f < m_Positions[v].size(); f++)
        {
            if (m_Annotations[v][f])
            {
                int ftp, ffn, ffp;
                
                if (f >= groundtruth.m_Positions[v].size())
                    getFrameSegmentationResults(vector<vector<pcl::PointXYZ> >(), m_Positions[v][f][0], ftp, ffn, ffp);
                else if (f >= m_Positions[v].size())
                    getFrameSegmentationResults(groundtruth.m_Positions[v][f], vector<pcl::PointXYZ>(), ftp, ffn, ffp);
                else
                    getFrameSegmentationResults(groundtruth.m_Positions[v][f], m_Positions[v][f][0], ftp, ffn, ffp);

                tp += ftp;
                fn += ffn;
                fp += ffp;
            }
        }
    }
}

void DetectionOutput::getRecognitionResults(DetectionOutput groundtruth, int& tp, int& fn, int& fp)
{
    assert( m_NumOfViews == m_Positions.size() );
    int gtsize = groundtruth.m_Positions.size();
    assert( m_NumOfViews == gtsize );
    //for (int i = 0; i < m_Positions.size(); i++)
    //    assert( m_NumOfFrames[i] == m_Positions[i].size() == groundtruth.m_Positions[i].size() );
    
    tp = fn = fp = 0;
    
    for (int v = 0; v < m_NumOfViews; v++)
    {
        for (int f = 0; f < groundtruth.m_Positions[v].size() && f < m_Positions[v].size(); f++)
        {
            if (m_Annotations[v][f])
            {
                if (f >= groundtruth.m_Positions[v].size())
                    getRecognitionFrameResults(vector<vector<pcl::PointXYZ> >(), m_Positions[v][f], tp, fn, fp);
                else if (f >= m_Positions[v].size())
                    getRecognitionFrameResults(groundtruth.m_Positions[v][f], vector<vector<pcl::PointXYZ> >(), tp, fn, fp);
                else
                    getRecognitionFrameResults(groundtruth.m_Positions[v][f], m_Positions[v][f], tp, fn, fp);
            }
        }
    }
}

void DetectionOutput::getFrameSegmentationResults(vector<int> indices, vector<vector<pcl::PointXYZ> > predictions, cv::Mat& errors)
{
    errors.create(indices.size(), 1, CV_32SC(3));
    
    for (int v = 0; v < indices.size(); v++)
    {
        vector<vector<pcl::PointXYZ> > annotations = m_Positions[v][indices[v]];
        
        int tp, fn, fp;
        getFrameSegmentationResults(annotations, predictions[v], tp, fn, fp);
        
        errors.at<cv::Vec3i>(v,0) = cv::Vec3i(tp, fn, fp);
    }
}

void DetectionOutput::getFrameSegmentationResults(vector<int> indices, vector<vector<pcl::PointXYZ> > predictions, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > >& correspondences, vector<vector<pair<pcl::PointXYZ, pcl::PointXYZ> > >& rejections, cv::Mat& errors)
{
    correspondences.resize(indices.size());
    rejections.resize(indices.size());
    errors.create(indices.size(), 1, CV_32SC(3));
    
    for (int v = 0; v < indices.size(); v++)
    {
        vector<vector<pcl::PointXYZ> > annotations = m_Positions[v][indices[v]];
        
        int tp, fn, fp;
        getFrameSegmentationResults(annotations, predictions[v], correspondences[v], rejections[v], tp, fn, fp);
        
        errors.at<cv::Vec3i>(v,0) = cv::Vec3i(tp, fn, fp);
    }
}

//void DetectionOutput::getSegmentationFrameResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<vector<pcl::PointXYZ> > predictions, int& tp, int& fn, int& fp)
//{
//    int ftp = 0, ffn = 0, ffp = 0; // frame tp, fn, and fp
//    
//    vector<vector<bool> > assignations;
//    for (int o = 0; o < predictions.size(); o++)
//        assignations.push_back(vector<bool>(predictions[o].size(), false));
//    
//    for (int o = 0; o < groundtruth.size(); o++)
//    {
//        for (int i = 0; i < groundtruth[o].size(); i++)
//        {
//            bool found = false;
//            
//            for (int k = 0; k < predictions.size() && !found; k++)
//            {
//                for (int j = 0; j < predictions[k].size() && !found; j++)
//                {
//                    pcl::PointXYZ p1 = groundtruth[o][i];
//                    pcl::PointXYZ p2 = predictions[k][j];
//                    
//                    float dist = distance(p1, p2);
//                    if (!assignations[k][j] && dist < m_Tol)
//                        found = assignations[k][j] = true;
//                }
//            }
//            found ? ftp++ : ffn++;
//        }
//    }
//    
//    for (int o = 0; o < assignations.size(); o++)
//        for (int i = 0; i < assignations[o].size(); i++)
//            if (!assignations[o][i]) ffp++;
//    
//    tp += ftp;
//    fn += ffn;
//    fp += ffp;
//}

void DetectionOutput::getRecognitionFrameResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<vector<pcl::PointXYZ> > predictions, int& tp, int& fn, int& fp)
{
    int ftp = 0, ffn = 0, ffp = 0;
    
    vector<vector<bool> > assignations;
    for (int o = 0; o < m_NumOfObjects; o++)
        assignations.push_back(vector<bool>(predictions[o].size(), false));
    
    for (int o = 1; o < m_NumOfObjects; o++) // there is not the 0 class in the groundtruth
    {
        for (int i = 0; i < groundtruth[o].size(); i++)
        {
            bool found = false;

            for (int j = 0; j < predictions[o].size() && !found; j++)
            {
                pcl::PointXYZ p1 = groundtruth[o][i];
                pcl::PointXYZ p2 = predictions[o][j];
                
                if (!assignations[o][j] && distance(p1, p2) < m_Tol)
                    found = assignations[o][j] = true;
            }
            found ? ftp++ : ffn++;
        }
    }
    
    for (int o = 1; o < m_NumOfObjects; o++)
        for (int i = 0; i < assignations[o].size(); i++)
            if (!assignations[o][i]) ffp++;
    
    tp += ftp;
    fn += ffn;
    fp += ffp;
}

//void DetectionOutput::greedyBlindAssignation(vector<vector<pcl::PointXYZ> > predictions, vector<vector<pcl::PointXYZ> > groundtruth)
//{
//    vector<vector<float> > distances;
//    
//    for (int p = 0; p < predictions.size(); p++) for (int i = 0; i < predictions[p].size(); i++)
//    {
//        vector<float> row;
//        pcl::PointXYZ q1 = predictions[p][i];
//        for (int g = 0; g < groundtruth.size(); g++) for (int j = 0; j < groundtruth[g].size(); j++)
//        {
//            pcl::PointXYZ q2 = groundtruth[g][j];
//            row.push_back( distance(q1,q2) );
//        }
//        distances.push_back(row);
//    }
//    
//    cv::Mat distancesMat;
//    cvx::convert(distances, distancesMat);
//    
//    cv::Mat sortedDistancesMat, I;
//    cv::Mat assignations (1, distancesMat.cols, cv::DataType<uchar>::type, cv::Scalar(0));
//    cvx::sortIdx(distancesMat, I, CV_SORT_EVERY_COLUMN | CV_SORT_ASCENDING, sortedDistancesMat);
//    for (int j = 0; j < I.cols; j++)
//    {
//        double minVal, maxVal;
//        cv::Point minIdx, maxIdx;
//        
//        cv::Mat row = sortedDistancesMat.row(j);
//        cv::minMaxLoc(row, &minVal, &maxVal, &minIdx, &maxIdx, (~assignations) & (row < m_Tol));
//        
//        assignations.at<uchar>(0, minIdx.x) = 255;
//    }
//}

void DetectionOutput::getFrameSegmentationResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<pcl::PointXYZ> predictions, int& tp, int& fn, int& fp)
{
    vector<vector<float> > distances;
    
    for (int k = 0; k < predictions.size(); k++)
    {
        vector<float> row;
        pcl::PointXYZ q1 = predictions[k];
        for (int o = 0; o < groundtruth.size(); o++) for (int i = 0; i < groundtruth[o].size(); i++)
        {
            pcl::PointXYZ q2 = groundtruth[o][i];
            row.push_back( distance(q1,q2) );
        }
        distances.push_back(row);
    }
    
    cv::Mat D;
    cvx::convert<float>(distances, D);
    
    tp = 0;
    if (D.empty())
    {
        fn = fp = 0;
        
        for (int o = 0; o < groundtruth.size(); o++)
            for (int i = 0; i < groundtruth[o].size(); i++)
                fn++;
        
        for (int k = 0; k < predictions.size(); k++)
            fp++;
    }
    else
    {
        cv::Mat M (D.size(), CV_8U, cv::Scalar(0)); // matches
        cv::Mat A = M.clone(); // assignations
        
        bool bRemain = true;
        for (int i = 0; i < D.rows && i < D.cols && bRemain; i++)
        {
            double minVal, maxVal;
            cv::Point minIdx, maxIdx;
            cv::minMaxLoc(D, &minVal, &maxVal, &minIdx, &maxIdx, ~A);
            
            if (minIdx.x == -1 && minIdx.y == -1)
            {
                bRemain = false;
            }
            else
            {
                M.at<uchar>(minIdx.y, minIdx.x) = 255;
                A.col(minIdx.x) = 255;
                A.row(minIdx.y) = 255;
            }
        }
        
        // Get the matches indices
        std::vector<cv::Point> matchesIndices;
        cv::findNonZero(M & (D <= m_Tol), matchesIndices);
        
        int tp = matchesIndices.size();
        fn = D.cols - tp;
        fp = D.rows - tp;
    }
    
    return;
}

void DetectionOutput::getFrameSegmentationResults(vector<vector<pcl::PointXYZ> > groundtruth, vector<pcl::PointXYZ> predictions, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& correspondences, vector<pair<pcl::PointXYZ, pcl::PointXYZ> >& rejections, int& tp, int& fn, int& fp)
{
    vector<vector<float> > distances;
    vector<pcl::PointXYZ> srlgroundtruth; // easier to index later
    
    for (int o = 0; o < groundtruth.size(); o++) for (int i = 0; i < groundtruth[o].size(); i++)
        srlgroundtruth.push_back(groundtruth[o][i]);

    for (int k = 0; k < predictions.size(); k++)
    {
        vector<float> row;
        for (int o = 0; o < srlgroundtruth.size(); o++)
            row.push_back( distance(predictions[k],srlgroundtruth[o]) );
        
        distances.push_back(row);
    }
    
    cv::Mat D;
    cvx::convert<float>(distances, D);
    
    tp = 0;
    if (D.empty())
    {
        fn = fp = 0;
        
        for (int o = 0; o < groundtruth.size(); o++)
            for (int i = 0; i < groundtruth[o].size(); i++)
                fn++;
        
        for (int k = 0; k < predictions.size(); k++)
            fp++;
    }
    else
    {
        cv::Mat M (D.size(), CV_8U, cv::Scalar(0)); // matches
        cv::Mat A = M.clone(); // assignations
        
        bool bRemain = true;
        for (int i = 0; i < D.rows && i < D.cols && bRemain; i++)
        {
            double minVal, maxVal;
            cv::Point minIdx, maxIdx;
            cv::minMaxLoc(D, &minVal, &maxVal, &minIdx, &maxIdx, ~A);
            
            if (minIdx.x == -1 && minIdx.y == -1)
            {
                bRemain = false;
            }
            else
            {
                M.at<uchar>(minIdx.y, minIdx.x) = 255;
                A.col(minIdx.x) = 255;
                A.row(minIdx.y) = 255;
            }
        }
        
        // Get the matches indices
        std::vector<cv::Point> matchesIndices;
        cv::findNonZero(M, matchesIndices);
        
        // and determine which are the ones below the threshold
        cv::Mat d (D <= m_Tol);
        
        tp = 0; // ..those will be the true positives (correspondences)
        
        for (int i = 0; i < matchesIndices.size(); i++)
        {
            pair<pcl::PointXYZ, pcl::PointXYZ> match;
            vector<pcl::PointXYZ> predictionsAux = predictions;
            vector<pcl::PointXYZ> srlgroundtruthAux = srlgroundtruth;
            match.first = predictions[matchesIndices[i].y];
            match.second = srlgroundtruth[matchesIndices[i].x];
            
            if ( d.at<uchar>(matchesIndices[i].y, matchesIndices[i].x) )
            {
                correspondences.push_back(match);
                tp++;
            }
            else
            {
                rejections.push_back(match);
            }
        }
        
        fn = D.cols - tp;
        fp = D.rows - tp;
    }
    
    return;
}