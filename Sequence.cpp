//
//  Sequence.cpp
//  remedi
//
//  Created by Albert Clapés on 15/05/14.
//
//

#include "Sequence.h"

//
// Sequence
//

template<typename FrameT>
SequenceBase<FrameT>::SequenceBase()
{
}

template<typename FrameT>
SequenceBase<FrameT>::SequenceBase(int numOfViews)
{
    m_Paths.resize(numOfViews);
    m_Streams.resize(numOfViews);
    m_FrameCounter.resize(numOfViews);
    m_Delays.resize(numOfViews, 0);
}

template<typename FrameT>
SequenceBase<FrameT>::SequenceBase(vector<vector<string> > paths)
{
    m_Paths = paths;
    
    m_Streams.resize(m_Paths.size());
    m_FrameCounter.resize(m_Paths.size(), -1);
    m_Delays.resize(m_Paths.size(), 0);
}

template<typename FrameT>
SequenceBase<FrameT>::SequenceBase(const SequenceBase<FrameT>& rhs)
{
    *this = rhs;
}

template<typename FrameT>
SequenceBase<FrameT>& SequenceBase<FrameT>::operator=(const SequenceBase<FrameT>& rhs)
{
    if (this != &rhs)
    {
        m_Path = rhs.m_Path;
        m_Name = rhs.m_Name;
        m_Paths = rhs.m_Paths;
        m_Streams = rhs.m_Streams;
        m_FrameCounter = rhs.m_FrameCounter;
        m_Delays = rhs.m_Delays;
    }
    
    return *this;
}

template<typename FrameT>
string SequenceBase<FrameT>::getPath()
{
    return m_Path;
}

template<typename FrameT>
void SequenceBase<FrameT>::setPath(string path)
{
    m_Path = path;
}

template<typename FrameT>
string SequenceBase<FrameT>::getName()
{
    return m_Name;
}

template<typename FrameT>
void SequenceBase<FrameT>::setName(string name)
{
    m_Name = name;
}

template<typename FrameT>
int SequenceBase<FrameT>::getNumOfViews()
{
    return m_Paths.size();
}

template<typename FrameT>
void SequenceBase<FrameT>::setNumOfViews(int n)
{
    m_Paths.resize(n);
    m_FrameCounter.resize(n);
    m_Delays.resize(n,0);
}

template<typename FrameT>
void SequenceBase<FrameT>::setFramesPaths(vector<vector<string> > paths)
{
    m_Paths = paths;
    
    m_FrameCounter.resize(m_Paths.size(), -1);
    m_Delays.resize(m_Paths.size(), 0);
}

template<typename FrameT>
void SequenceBase<FrameT>::addStream(vector<typename FrameT::Ptr> stream)
{
    m_Streams.push_back(stream);
    m_FrameCounter.push_back(-1);
    
    m_Delays.resize(m_Streams.size());
}

template<typename FrameT>
void SequenceBase<FrameT>::setStream(vector<typename FrameT::Ptr> stream, int view)
{
    while (view >= m_Streams.size())
        m_Streams.push_back(vector<typename FrameT::Ptr>());
    
    m_Streams[view] = stream;
}

template<typename FrameT>
void SequenceBase<FrameT>::setStreams(vector<vector<typename FrameT::Ptr> > streams)
{
    m_Streams = streams;
    
    m_FrameCounter.resize(m_Streams.size(), -1);
    m_Delays.resize(m_Streams.size(), 0);
}

template<typename FrameT>
void SequenceBase<FrameT>::addFrameFilePath(string framePath, int view)
{
    while (view >= m_Paths.size())
    {
        m_Paths.push_back(vector<string>());
        m_Streams.push_back(vector<typename FrameT::Ptr>());
        m_FrameCounter.push_back(-1);
        
        m_Delays.push_back(0);
    }
    
    m_Paths[view].push_back(framePath);
}

template<typename FrameT>
void SequenceBase<FrameT>::addFrameFilePath(vector<string> framePaths)
{
    for (int i = 0; i < m_Paths.size(); i++)
        m_Paths[i].push_back(framePaths[i]);
}

template<typename FrameT>
void SequenceBase<FrameT>::addFrame(typename FrameT::Ptr frame, int view)
{
    m_Streams[view].push_back(frame);
}

template<typename FrameT>
void SequenceBase<FrameT>::addFrame(vector<typename FrameT::Ptr> frame)
{
    for (int i = 0; i < m_Streams.size(); i++)
        m_Streams[i].push_back(frame[i]);
}

template<typename FrameT>
bool SequenceBase<FrameT>::hasNextFrames(int step)
{
    for (int i = 0; i < m_Paths.size(); i++)
        if (m_FrameCounter[i] + m_Delays[i] + step > m_Paths[i].size() - 1)
            return false;
    
    return true;
}

template<typename FrameT>
bool SequenceBase<FrameT>::hasPreviousFrames(int step)
{
    for (int i = 0; i < m_Paths.size(); i++)
        if (m_FrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

template<typename FrameT>
vector<typename FrameT::Ptr> SequenceBase<FrameT>::nextFrames(int step)
{
    for (int v = 0; v < m_Paths.size(); v++)
        m_FrameCounter[v] += step;
    
    vector<typename FrameT::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        typename FrameT::Ptr frame (new FrameT);
        int delay = (m_Delays.size()) > 0 ? m_Delays[v] : 0;
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][m_FrameCounter[v] + delay]; // preallocation
        else
            readFrame(m_Paths[v], m_FrameCounter[v] + delay, *frame);
        
        frames.push_back(frame);
    }

    return frames;
}

template<typename FrameT>
vector<typename FrameT::Ptr> SequenceBase<FrameT>::previousFrames(int step)
{
    for (int v = 0; v < m_Paths.size(); v++)
        m_FrameCounter[v] -= step;
    
    vector<typename FrameT::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        typename FrameT::Ptr frame (new FrameT);
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][m_FrameCounter[v] + m_Delays[v]]; // preallocation
        else
            readFrame(m_Paths[v], m_FrameCounter[v] + m_Delays[v], *frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

template<typename FrameT>
vector<typename FrameT::Ptr> SequenceBase<FrameT>::getFrames(int i)
{
    vector<typename FrameT::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        typename FrameT::Ptr frame (new FrameT);
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][i];
        else
            readFrame(m_Paths[v], i, *frame);
        
        frames.push_back(frame);
    }
    
    return frames;
}

template<typename FrameT>
vector<int> SequenceBase<FrameT>::getNumOfFrames()
{
    vector<int> numOfFrames;
    for (int i = 0; i < m_Paths.size(); i++)
        numOfFrames.push_back(m_Paths[i].size());
    
    return numOfFrames;
}

template<typename FrameT>
vector<int> SequenceBase<FrameT>::getCurrentFramesID()
{
    vector<int> counters;
    for (int i = 0; i < m_Paths.size(); i++)
        counters.push_back(m_FrameCounter[i] + m_Delays[i]);
    
    return counters;
}

template<typename FrameT>
vector<float> SequenceBase<FrameT>::getProgress()
{
    vector<float> progresses;
    for (int i = 0; i < m_Paths.size(); i++)
        progresses.push_back(((float) (m_FrameCounter[i] + m_Delays[i])) / m_Paths[i].size());
    
    return progresses;
}

template<typename FrameT>
vector<int> SequenceBase<FrameT>::at()
{
    vector<int> frameDelayedCounter(m_Streams.size());
    
    for (int i = 0; i < m_Streams.size(); i++)
        frameDelayedCounter[i] = m_FrameCounter[i] + m_Delays[i];
    
    return frameDelayedCounter;
}

template<typename FrameT>
void SequenceBase<FrameT>::setDelays(vector<int> delays)
{
    m_Delays = delays;
}

template<typename FrameT>
void SequenceBase<FrameT>::restart()
{
    m_FrameCounter.clear();
    m_FrameCounter.resize(m_Streams.size(), -1);
}

template<typename FrameT>
void SequenceBase<FrameT>::readFrame(vector<string> paths, int f, FrameT& frame)
{
    assert (f < paths.size());
    
    frame = FrameT( cv::imread(paths[f].c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR) );
}

template<typename FrameT>
void SequenceBase<FrameT>::allocate()
{
    m_Streams.resize(m_Paths.size());
    
    for (int v = 0; v < m_Paths.size(); v++)
    {
        for (int f = 0; f < m_Paths[v].size(); f++)
        {
            typename FrameT::Ptr frame (new FrameT);
            
            int flags = CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR;
            frame->set( cv::imread(m_Paths[v][f], flags) );
            
            m_Streams[v].push_back(frame);
        }
    }
}

// Sequence<DepthFrame>

Sequence<DepthFrame>::Sequence()
{
}

Sequence<DepthFrame>::Sequence(int numOfViews)
: SequenceBase<DepthFrame>(numOfViews)
{
}

Sequence<DepthFrame>::Sequence(vector< vector<string> > paths)
: SequenceBase<DepthFrame>(paths)
{
}

Sequence<DepthFrame>::Sequence(const Sequence& rhs)
: SequenceBase(rhs)
{
    *this = rhs;
}

Sequence<DepthFrame>& Sequence<DepthFrame>::operator=(const Sequence<DepthFrame>& rhs)
{
    if (this != &rhs)
    {
        SequenceBase<DepthFrame>::operator=(rhs);
        m_ReferencePoints = rhs.m_ReferencePoints;
        m_Transformations = rhs.m_Transformations;
    }
    
    return *this;
}

void Sequence<DepthFrame>::setReferencePoints(vector<pcl::PointXYZ> points)
{
    m_ReferencePoints = points;
}

vector<pcl::PointXYZ> Sequence<DepthFrame>::getReferencePoints()
{
    return m_ReferencePoints;
}

void Sequence<DepthFrame>::setRegistrationTransformations(vector<Eigen::Matrix4f> transformations)
{
    m_Transformations = transformations;
}

//
// Sequence<ColorDepthFrame>
//

Sequence<ColorDepthFrame>::Sequence()
{
}

Sequence<ColorDepthFrame>::Sequence(int numOfViews)
{
    m_Paths.resize(numOfViews);
    m_Streams.resize(numOfViews);
    m_FrameCounter.resize(numOfViews);
    m_Delays.resize(numOfViews, 0);
}

Sequence<ColorDepthFrame>::Sequence(vector<vector< pair<string,string> > > paths)
{
    m_Paths = paths;
    
    m_Streams.resize(m_Paths.size());
    m_FrameCounter.resize(m_Paths.size(), -1);
    m_Delays.resize(m_Paths.size(), 0);
}

Sequence<ColorDepthFrame>::Sequence(const Sequence& rhs)
{
    *this = rhs;
}

Sequence<ColorDepthFrame>& Sequence<ColorDepthFrame>::operator=(const Sequence<ColorDepthFrame>& rhs)
{
    if (this != &rhs)
    {
        m_Path = rhs.m_Path;
        m_Name = rhs.m_Name;
        m_Paths = rhs.m_Paths;
        m_Streams = rhs.m_Streams;
        m_FrameCounter = rhs.m_FrameCounter;
        m_Delays = rhs.m_Delays;
        
        m_ReferencePoints = rhs.m_ReferencePoints;
        m_Transformations = rhs.m_Transformations;
    }
    
    return *this;
}

string Sequence<ColorDepthFrame>::getPath()
{
    return m_Path;
}

void Sequence<ColorDepthFrame>::setPath(string path)
{
    m_Path = path;
}

string Sequence<ColorDepthFrame>::getName()
{
    return m_Name;
}

void Sequence<ColorDepthFrame>::setName(string name)
{
    m_Name = name;
}

int Sequence<ColorDepthFrame>::getNumOfViews()
{
    return m_Paths.size();
}

void Sequence<ColorDepthFrame>::setNumOfViews(int n)
{
    m_Paths.resize(n);
    m_FrameCounter.resize(n);
    m_Delays.resize(n,0);
}

void Sequence<ColorDepthFrame>::setFramesPaths(vector<vector< pair<string,string> > > paths)
{
    m_Paths = paths;
    
    m_FrameCounter.resize(m_Paths.size(), -1);
    m_Delays.resize(m_Paths.size(), 0);
}

void Sequence<ColorDepthFrame>::addStream(vector<ColorDepthFrame::Ptr> stream)
{
    m_Streams.push_back(stream);
    m_FrameCounter.push_back(-1);
    
    m_Delays.resize(m_Streams.size());
}

void Sequence<ColorDepthFrame>::setStream(vector<ColorDepthFrame::Ptr> stream, int view)
{
    while (view >= m_Streams.size())
        m_Streams.push_back(vector<ColorDepthFrame::Ptr>());
    
    m_Streams[view] = stream;
}

void Sequence<ColorDepthFrame>::setStreams(vector<vector<ColorDepthFrame::Ptr> > streams)
{
    m_Streams = streams;
    
    m_FrameCounter.resize(m_Streams.size(), -1);
    m_Delays.resize(m_Streams.size(), 0);
}

void Sequence<ColorDepthFrame>::addFramesFilePath(string colorPath, string depthPath, int view)
{
    while (view >= m_Paths.size())
    {
        m_Paths.push_back(vector< pair<string,string> >());
        m_Streams.push_back(vector<ColorDepthFrame::Ptr>());
        m_FrameCounter.push_back(-1);
        
        m_Delays.push_back(0);
    }
    
    m_Paths[view].push_back( pair<string,string>(colorPath,depthPath) );
}

void Sequence<ColorDepthFrame>::addFrameFilePath(vector< pair<string,string> > framePaths)
{
    for (int i = 0; i < m_Paths.size(); i++)
        m_Paths[i].push_back(framePaths[i]);
}

void Sequence<ColorDepthFrame>::addFrame(ColorDepthFrame::Ptr frame, int view)
{
    m_Streams[view].push_back(frame);
}

void Sequence<ColorDepthFrame>::addFrame(vector<ColorDepthFrame::Ptr> frame)
{
    for (int i = 0; i < m_Streams.size(); i++)
        m_Streams[i].push_back(frame[i]);
}

bool Sequence<ColorDepthFrame>::hasNextFrames(int step)
{
    for (int i = 0; i < m_Paths.size(); i++)
        if (m_FrameCounter[i] + m_Delays[i] + step > m_Paths[i].size() - 1)
            return false;
    
    return true;
}

bool Sequence<ColorDepthFrame>::hasPreviousFrames(int step)
{
    for (int i = 0; i < m_Paths.size(); i++)
        if (m_FrameCounter[i] + m_Delays[i] - step < 0)
            return false;
    
    return true;
}

vector<ColorDepthFrame::Ptr> Sequence<ColorDepthFrame>::nextFrames(int step)
{
    for (int v = 0; v < m_Paths.size(); v++)
        m_FrameCounter[v] += step;
    
    vector<ColorDepthFrame::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        ColorDepthFrame::Ptr frame (new ColorDepthFrame);
        int delay = (m_Delays.size()) > 0 ? m_Delays[v] : 0;
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][m_FrameCounter[v] + delay]; // preallocation
        else
            readFrame(m_Paths[v], m_FrameCounter[v] + delay, *frame);
        
        if (v < m_Transformations.size())
        {
            frame->setReferencePoint(m_ReferencePoints[v]);
            frame->setRegistrationTransformation(m_Transformations[v]);
        }
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<ColorDepthFrame::Ptr> Sequence<ColorDepthFrame>::previousFrames(int step)
{
    for (int v = 0; v < m_Paths.size(); v++)
        m_FrameCounter[v] -= step;
    
    vector<ColorDepthFrame::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        ColorDepthFrame::Ptr frame (new ColorDepthFrame);
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][m_FrameCounter[v] + m_Delays[v]]; // preallocation
        else
            readFrame(m_Paths[v], m_FrameCounter[v] + m_Delays[v], *frame);
        
        if (v < m_Transformations.size())
        {
            frame->setReferencePoint(m_ReferencePoints[v]);
            frame->setRegistrationTransformation(m_Transformations[v]);
        }
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<ColorDepthFrame::Ptr> Sequence<ColorDepthFrame>::getFrames(int i)
{
    vector<ColorDepthFrame::Ptr> frames;
    for (int v = 0; v < m_Paths.size(); v++)
    {
        ColorDepthFrame::Ptr frame (new ColorDepthFrame);
        if (m_Streams[v].size() > 0)
            frame = m_Streams[v][i];
        else
            readFrame(m_Paths[v], i, *frame);
        
        if (v < m_Transformations.size())
        {
            frame->setReferencePoint(m_ReferencePoints[v]);
            frame->setRegistrationTransformation(m_Transformations[v]);
        }
        
        frames.push_back(frame);
    }
    
    return frames;
}

vector<int> Sequence<ColorDepthFrame>::getNumOfFrames()
{
    vector<int> numOfFrames;
    for (int i = 0; i < m_Paths.size(); i++)
        numOfFrames.push_back(m_Paths[i].size());
    
    return numOfFrames;
}

vector<int> Sequence<ColorDepthFrame>::getCurrentFramesID()
{
    vector<int> counters;
    for (int i = 0; i < m_Paths.size(); i++)
        counters.push_back(m_FrameCounter[i] + m_Delays[i]);
    
    return counters;
}

vector<float> Sequence<ColorDepthFrame>::getProgress()
{
    vector<float> progresses;
    for (int i = 0; i < m_Paths.size(); i++)
        progresses.push_back(((float) (m_FrameCounter[i] + m_Delays[i])) / m_Paths[i].size());
    
    return progresses;
}

vector<int> Sequence<ColorDepthFrame>::at()
{
    vector<int> frameDelayedCounter(m_Streams.size());
    
    for (int i = 0; i < m_Streams.size(); i++)
        frameDelayedCounter[i] = m_FrameCounter[i] + m_Delays[i];
    
    return frameDelayedCounter;
}

void Sequence<ColorDepthFrame>::setDelays(vector<int> delays)
{
    m_Delays = delays;
}

void Sequence<ColorDepthFrame>::setReferencePoints(vector<pcl::PointXYZ> points)
{
    m_ReferencePoints = points;
}

vector<pcl::PointXYZ> Sequence<ColorDepthFrame>::getReferencePoints()
{
    return m_ReferencePoints;
}

void Sequence<ColorDepthFrame>::setRegistrationTransformations(vector<Eigen::Matrix4f> transformations)
{
    m_Transformations = transformations;
}

void Sequence<ColorDepthFrame>::restart()
{
    m_FrameCounter.clear();
    m_FrameCounter.resize(m_Streams.size(), -1);
}

void Sequence<ColorDepthFrame>::allocate()
{
    m_Streams.resize(m_Paths.size());
    
    for (int v = 0; v < m_Paths.size(); v++)
    {
        for (int f = 0; f < m_Paths[v].size(); f++)
        {
            ColorDepthFrame::Ptr frame (new ColorDepthFrame);
            
            int flags = CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR;
            frame->set( cv::imread(m_Paths[v][f].first, flags), cv::imread(m_Paths[v][f].second, flags) );
            
            m_Streams[v].push_back(frame);
        }
    }
}

void Sequence<ColorDepthFrame>::readFrame(vector< pair<string,string> > paths, int f, ColorDepthFrame& frame)
{
    assert (f < paths.size());
    
    int flags = CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR;
    frame = ColorDepthFrame( cv::imread(paths[f].first, flags), cv::imread(paths[f].second, flags) );
}

//
// Template instanciation
// ------------------------------------------------------
template class SequenceBase<Frame>;
template class SequenceBase<ColorFrame>;
template class SequenceBase<DepthFrame>;

template class Sequence<DepthFrame>;
// ------------------------------------------------------
