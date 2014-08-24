#pragma once

#include "DepthFrame.h"
#include "KinectReader.h"
#include "Sequence.h"

#include <opencv2/opencv.hpp>

template<typename FrameT>
class BackgroundSubtractorBase
{
public:
    BackgroundSubtractorBase();
    BackgroundSubtractorBase(const BackgroundSubtractorBase<FrameT>& rhs);
    
    BackgroundSubtractorBase& operator=(const BackgroundSubtractorBase<FrameT>& rhs);
    
    void setFramesResolution(int xres, int yres);
    
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);
    void setVarGenThreshold(float var);
    void setMorphologyLevel(int level = 0);
    
    int getNumOfSamples();
    int getNumOfMixtureComponents();
    float getLearningRate();
    float getBackgroundRatio();
    float getVarGenThreshold();
    int getMorphologyLevel();

    void setInputFrames(vector<typename FrameT::Ptr> frames);
    
    // Measure overlaps of the sequence views
//    template<typename T>
//    bool measureOverlap(typename Sequence<T>::Ptr pValSeq, vector<float>& seqOverlaps);
    
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
    
    int m_XRes;
    int m_YRes;
    
    int m_NumOfSamples;
    int m_NumOfMixtureComponents;
    float m_LearningRate;
    float m_BackgroundRatio;
    float m_VarGenThreshold;
    int m_MorphLevel;
    
    bool m_bRecompute;
    
    vector<cv::Mat> m_MeanFrames;
    vector<cv::Mat> m_StddevFrames;
    vector<cv::BackgroundSubtractorGMG> m_Subtractors;

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
    BackgroundSubtractor(const BackgroundSubtractor<ColorFrame>& rhs);
    
    BackgroundSubtractor<ColorFrame>& operator=(const BackgroundSubtractor<ColorFrame>& rhs);
    
    void setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<ColorFrame::Ptr> frames);
    vector<ColorFrame::Ptr> getInputFrames() const;

    void setShadowsModeling(bool shadowsModeling = true);
    
    void model();
    
    void subtract(vector<ColorFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<ColorFrame> > Ptr;
    
protected:
    void subtract(int v, ColorFrame& fgFrame);
    
private:
    void subtract(cv::BackgroundSubtractorGMG& subtractor, ColorFrame::Ptr frame,ColorFrame& fgFrame);
    
    Sequence<ColorFrame>::Ptr m_pSeq;
    vector<ColorFrame::Ptr> m_InputFrames;
    bool m_bShadowsModeling;
};

template<>
class BackgroundSubtractor<DepthFrame> : public BackgroundSubtractorBase<DepthFrame>
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor<DepthFrame>& rhs);
    
    BackgroundSubtractor<DepthFrame>& operator=(const BackgroundSubtractor<DepthFrame>& rhs);
    
    void setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<DepthFrame::Ptr> frames);
    vector<DepthFrame::Ptr> getInputFrames() const;
    
    void model();
    
    void subtract(vector<DepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<DepthFrame> > Ptr;
    
protected:
    void subtract(int v, DepthFrame& fgFrame);
    
private:
    void subtract(cv::BackgroundSubtractorGMG& subtractor, DepthFrame::Ptr frame, DepthFrame& fgFrame);
    
    Sequence<DepthFrame>::Ptr m_pSeq;
    vector<DepthFrame::Ptr> m_InputFrames;
};

template<>
class BackgroundSubtractor<ColorDepthFrame> : public BackgroundSubtractor<ColorFrame>, public BackgroundSubtractor<DepthFrame>
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor<ColorDepthFrame>& rhs);
    
    BackgroundSubtractor<ColorDepthFrame>& operator=(const BackgroundSubtractor<ColorDepthFrame>& rhs);
    
    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    
    void setFramesResolution(int xres, int yres);
    
    void setModality(int m);
    int getModality() const;
    
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);
    void setVarGenThreshold(float var);
    void setMorphologyLevel(int level = 0);
    
    void model();
    void subtract(vector<ColorDepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<ColorDepthFrame> > Ptr;
    
private:
    
    //
    // Members
    //
    
    int m_Modality;
};
