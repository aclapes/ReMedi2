#pragma once

#include "DepthFrame.h"
#include "Reader.h"
#include "Sequence.h"

#include <opencv2/opencv.hpp>

template<typename FrameT>
class BackgroundSubtractorBase
{
public:
    BackgroundSubtractorBase();
    BackgroundSubtractorBase(const BackgroundSubtractorBase& rhs);
	~BackgroundSubtractorBase(void);
    
    BackgroundSubtractorBase& operator=(const BackgroundSubtractorBase& rhs);
	void operator()(DepthFrame, float alpha = 0);
	void operator()(DepthFrame, DepthFrame, float alpha = 0);
    
    void setInputSequence(typename Sequence<FrameT>::Ptr pSeq);
    
    void setNumOfSamples(int n);
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);
    void setMorphologyLevel(int level = 0);
    
    int getNumOfSamples();
    int getNumOfMixtureComponents();
    float getLearningRate();
    float getBackgroundRatio();
    int getMorphologyLevel();

    // Measure overlaps of the sequence views
    template<typename T>
    bool measureOverlap(typename Sequence<T>::Ptr pValSeq, vector<float>& seqOverlaps);
    
    typedef boost::shared_ptr<BackgroundSubtractorBase<FrameT> > Ptr;

private:
    template<typename T>
    void accumulate(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts = cv::noArray());
    template<typename T>
    void accumulateSquareDifferences(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts = cv::noArray());
    
    void stddev(cv::InputArray sqdiffsacc, cv::InputArray count, cv::InputArray err, cv::OutputArray stddev);
    void standardize(cv::InputArray frame, cv::InputArray mean, cv::InputArray stddev, cv::InputArray errors, cv::OutputArray frameStd);
    
protected:
    //
    // Attributes
    //
    
    typename Sequence<FrameT>::Ptr m_pSeq;
    
    int m_NumOfSamples;
    int m_NumOfMixtureComponents;
    float m_LearningRate;
    float m_BackgroundRatio;
    int m_MorphLevel;
    
    vector<cv::Mat> m_MeanFrames;
    vector<cv::Mat> m_StddevFrames;
    vector<cv::BackgroundSubtractorMOG2> m_Subtractors;

    vector<cv::Mat> m_StationaryErrorsFrames;
};

template<typename FrameT>
class BackgroundSubtractor : public BackgroundSubtractorBase<FrameT>
{
public:
    typedef boost::shared_ptr<BackgroundSubtractor<FrameT> > Ptr;
};

template<>
class BackgroundSubtractor<ColorFrame> : public BackgroundSubtractorBase<ColorFrame>
{
public:
    BackgroundSubtractor();
    
    void model();
    void subtract(vector<ColorFrame::Ptr> frames, vector<ColorFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<ColorFrame> > Ptr;
    
private:
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, ColorFrame::Ptr frame, cv::Mat mean, cv::Mat stddev, ColorFrame& fgFrame);
};

template<>
class BackgroundSubtractor<DepthFrame> : public BackgroundSubtractorBase<DepthFrame>
{
public:
    BackgroundSubtractor();
    
    void model();
    void subtract(vector<DepthFrame::Ptr> frames, vector<DepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<DepthFrame> > Ptr;
    
private:
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, DepthFrame::Ptr frame, cv::Mat mean, cv::Mat stddev, cv::Mat errors, DepthFrame& fgFrame);
};

template<>
class BackgroundSubtractor<ColorDepthFrame> : public BackgroundSubtractorBase<ColorDepthFrame>
{
public:
    BackgroundSubtractor();
    
    void model();
    void subtract(vector<ColorDepthFrame::Ptr> frames, vector<ColorDepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<ColorDepthFrame> > Ptr;
    
private:
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, ColorDepthFrame::Ptr frame, cv::Mat mean, cv::Mat stddev, cv::Mat errors, ColorDepthFrame& fgFrame);
};
