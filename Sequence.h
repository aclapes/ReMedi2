//
//  Sequence.h
//  remedi
//
//  Created by Albert Clapés on 15/05/14.
//
//

#ifndef __remedi__Sequence__
#define __remedi__Sequence__

#include "ColorFrame.h"
#include "DepthFrame.h"
#include "ColorDepthFrame.h"

#include <iostream>
#include <vector>

using namespace std;

template<typename FrameT>
class Sequence
{
public:
    Sequence();
    Sequence(int numOfViews);
    Sequence(vector<vector<string> > paths);
    Sequence(vector<vector<typename FrameT::Ptr> > stream);
    Sequence(const Sequence& rhs);

    Sequence& operator=(const Sequence& rhs);
    
    string getPath();
    void setPath(string path);

    string getName();
    void setName(string name);
    
    int getNumOfViews();
    void setNumOfViews(int n);
    
    void setFramesPaths(vector<vector<string> > paths);
    
    void addStream(vector<typename FrameT::Ptr> stream);
    void setStream(vector<typename FrameT::Ptr> stream, int view);
    void setStreams(vector<vector<typename FrameT::Ptr> > streams);
    
    void addFrameFilePath(string framePath, int view);
    void addFrameFilePath(vector<string> framePaths);
    
    void addFrame(typename FrameT::Ptr frame, int view);
    void addFrame(vector<typename FrameT::Ptr> frames);
    
    bool hasNextFrames(int step = 1);
    bool hasPreviousFrames(int step = 1);
    
    vector<typename FrameT::Ptr> nextFrames(int step = 1);
    vector<typename FrameT::Ptr> previousFrames(int step = 1);
    vector<typename FrameT::Ptr> getFrames(int i);
    
    vector<int> getNumOfFrames();
    vector<int> getCurrentFramesID();
    vector<float> getProgress();
    vector<int> at();
    
    void setDelays(vector<int> delays);
    
    void restart();
    
    void allocate();
    
    typedef boost::shared_ptr<Sequence<FrameT> > Ptr;
    
private:
    void readFrame(vector<string> paths, int i, FrameT& frame);
    
    //
    // Attributes
    //
    
    string m_Path;
    string m_Name;
    
    vector< vector<string> > m_Paths;
    vector< vector<typename FrameT::Ptr> > m_Streams;
    
    vector<int> m_FrameCounter;
    vector<int> m_Delays;
};

template<>
class Sequence<ColorDepthFrame>
{
public:
    Sequence();
    Sequence(int numOfViews);
    Sequence(vector< vector< pair<string,string> > > paths);
    Sequence(vector<vector<ColorDepthFrame::Ptr> > stream);
    Sequence(const Sequence& rhs);

    Sequence& operator=(const Sequence& rhs);
    
    string getPath();
    void setPath(string path);
    
    string getName();
    void setName(string name);
    
    int getNumOfViews();
    void setNumOfViews(int n);
    
    void setFramesPaths(vector< vector< pair<string,string> > > paths);
    
    void addStream(vector<ColorDepthFrame::Ptr> stream);
    void setStream(vector<ColorDepthFrame::Ptr> stream, int view);
    void setStreams(vector<vector<ColorDepthFrame::Ptr> > streams);
    
    void addFramesFilePath(string colorPath, string depthPath, int view);
    void addFrameFilePath(vector< pair<string,string> > framePaths);
    
    void addFrame(ColorDepthFrame::Ptr frame, int view);
    void addFrame(vector<ColorDepthFrame::Ptr> frames);
    
    bool hasNextFrames(int step = 1);
    bool hasPreviousFrames(int step = 1);
    
    vector<ColorDepthFrame::Ptr> nextFrames(int step = 1);
    vector<ColorDepthFrame::Ptr> previousFrames(int step = 1);
    vector<ColorDepthFrame::Ptr> getFrames(int i);
    
    vector<int> getNumOfFrames();
    vector<int> getCurrentFramesID();
    vector<float> getProgress();
    vector<int> at();
    
    void setDelays(vector<int> delays);
    
    void restart();
    
    void allocate();
    
    typedef boost::shared_ptr<Sequence<ColorDepthFrame> > Ptr;
    
private:
    void readFrame(vector< pair<string,string> > paths, int i, ColorDepthFrame& frame);
    
    //
    // Attributes
    //
    
    string m_Path;
    string m_Name;
    
    vector< vector< pair<string,string> > > m_Paths;
    vector< vector<ColorDepthFrame::Ptr> > m_Streams;
    
    vector<int> m_FrameCounter;
    vector<int> m_Delays;
};

#endif /* defined(__remedi__Sequence__) */
