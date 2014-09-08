#pragma once

#include "DepthFrame.h"
#include "KinectReader.h"
#include "Sequence.h"

#include <opencv2/opencv.hpp>

template<typename SubtractorT, typename FrameT>
class BackgroundSubtractorBase
{
    
};

template<typename SubtractorT, typename FrameT>
class BackgroundSubtractor : public BackgroundSubtractorBase<SubtractorT, FrameT>
{
    typedef boost::shared_ptr<BackgroundSubtractor<SubtractorT, FrameT> > Ptr;
};

//// =============================================================================
////
//// BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, FrameT>
////
//// =============================================================================
//
//template<typename FrameT>
//class BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, FrameT>
//{
//public:
//    BackgroundSubtractorBase();
//    BackgroundSubtractorBase(const BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, FrameT>& rhs);
//    
//    BackgroundSubtractorBase& operator=(const BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, FrameT>& rhs);
//    
//    void setFramesResolution(int xres, int yres);
//    
//    void setNumOfMaxFeatures(int f);
//    void setLearningRate(float rate);
//    void setQuantizationLevels(int q);
//    void setDecisionThreshold(float t);
//    void setOpeningSize(int level = 0);
//    
//    int getNumOfSamples();
//    
//    int getNumOfMaxFeatures();
//    float getLearningRate();
//    int getQuantizationLevels();
//    float getDecisionThreshold();
//    int getOpeningSize();
//    
//    void setInputFrames(vector<typename FrameT::Ptr> frames);
//    
//    typedef boost::shared_ptr<BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, FrameT> > Ptr;
//    
//protected:
//    
//    int m_XRes;
//    int m_YRes;
//    
//    int m_NumOfSamples;
//    
//    int m_NumOfMaxFeatures;
//    float m_LearningRate;
//    int m_QuantizationLevels;
//    float m_DecisionThreshold;
//    int m_OpeningSize;
//    
//    bool m_bRecompute;
//    vector<cv::BackgroundSubtractorGMG> m_Subtractors;
//};
//
//template<>
//class BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame> : public BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, ColorFrame>
//{
//public:
//    BackgroundSubtractor();
//    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame>& rhs);
//    
//    BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame>& rhs);
//    
//    void setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n = 0);
//    void setInputFrames(vector<ColorFrame::Ptr> frames);
//    vector<ColorFrame::Ptr> getInputFrames() const;
//    
//    void setShadowsModeling(bool shadowsModeling = true);
//    
//    void model();
//    
//    void subtract(vector<ColorFrame::Ptr>& fgFrames);
//    
//    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame> > Ptr;
//    
//protected:
//    void subtract(int v, ColorFrame& fgFrame);
//    
//private:
//    void subtract(cv::BackgroundSubtractorGMG& subtractor, ColorFrame::Ptr frame,ColorFrame& fgFrame);
//    
//    Sequence<ColorFrame>::Ptr m_pSeq;
//    vector<ColorFrame::Ptr> m_InputFrames;
//    bool m_bShadowsModeling;
//};
//
//template<>
//class BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame> : public BackgroundSubtractorBase<cv::BackgroundSubtractorGMG, DepthFrame>
//{
//public:
//    BackgroundSubtractor();
//    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame>& rhs);
//    
//    BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame>& rhs);
//    
//    void setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n = 0);
//    void setInputFrames(vector<DepthFrame::Ptr> frames);
//    vector<DepthFrame::Ptr> getInputFrames() const;
//    
//    void model();
//    
//    void subtract(vector<DepthFrame::Ptr>& fgFrames);
//    
//    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame> > Ptr;
//    
//protected:
//    void subtract(int v, DepthFrame& fgFrame);
//    
//private:
//    void subtract(cv::BackgroundSubtractorGMG& subtractor, DepthFrame::Ptr frame, DepthFrame& fgFrame);
//    
//    Sequence<DepthFrame>::Ptr m_pSeq;
//    vector<DepthFrame::Ptr> m_InputFrames;
//};
//
//template<>
//class BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame> : public BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorFrame>, public BackgroundSubtractor<cv::BackgroundSubtractorGMG, DepthFrame>
//{
//public:
//    BackgroundSubtractor();
//    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>& rhs);
//    
//    BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame>& rhs);
//    
//    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pSeq, int n = 0);
//    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
//    
//    void setFramesResolution(int xres, int yres);
//    
//    void setModality(int m);
//    int getModality() const;
//    
//    void setNumOfMaxFeatures(int f);
//    void setLearningRate(float rate);
//    void setQuantizationLevels(int q);
//    void setDecisionThreshold(float t);
//    void setOpeningSize(int level = 0);
//    
//    void model();
//    void subtract(vector<ColorDepthFrame::Ptr>& fgFrames);
//    
//    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorGMG, ColorDepthFrame> > Ptr;
//    
//private:
//    void _model();
//    void subtract(int v, ColorDepthFrame& fgFrame);
//    void subtract(cv::BackgroundSubtractorGMG& subtractor, ColorDepthFrame::Ptr frame, ColorDepthFrame& fgFrame);
//    
//    int m_XRes;
//    int m_YRes;
//    
//    int m_NumOfSamples;
//    
//    int m_NumOfMaxFeatures;
//    float m_LearningRate;
//    int m_QuantizationLevels;
//    float m_DecisionThreshold;
//    int m_OpeningSize;
//    
//    bool m_bRecompute;
//    vector<cv::BackgroundSubtractorGMG> m_Subtractors;
//    
//    int m_Modality;
//    
//    Sequence<ColorDepthFrame>::Ptr m_pSeq;
//    vector<ColorDepthFrame::Ptr> m_InputFrames;
//};


// =============================================================================
//
// BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, FrameT>
//
// =============================================================================

template<typename FrameT>
class BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, FrameT>
{
public:
    BackgroundSubtractorBase();
    BackgroundSubtractorBase(const BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, FrameT>& rhs);
    
    BackgroundSubtractorBase& operator=(const BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, FrameT>& rhs);
    
    void setFramesResolution(int xres, int yres);
    
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);
    void setVarThresholdGen(float var);
    void setOpeningSize(int level = 0);
    
    int getNumOfSamples();
    
    int getNumOfMixtureComponents();
    float getLearningRate();
    float getBackgroundRatio();
    float getVarThresholdGen();
    int getOpeningSize();
    
    void setInputFrames(vector<typename FrameT::Ptr> frames);
    
    typedef boost::shared_ptr<BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, FrameT> > Ptr;
    
private:
    template<typename T>
    void accumulate(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts = cv::noArray());
    template<typename T>
    void accumulateSquareDifferences(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts = cv::noArray());
    
    void stddev(cv::InputArray sqdiffsacc, cv::InputArray count, cv::InputArray err, cv::OutputArray stddev);
    void standardize(cv::InputArray frame, cv::InputArray mean, cv::InputArray stddev, cv::InputArray errors, cv::OutputArray frameStd);
    
protected:
    
    int m_XRes;
    int m_YRes;
    
    int m_NumOfSamples;
    
    int m_NumOfMixtureComponents;
    float m_LearningRate;
    float m_BackgroundRatio;
    float m_VarThresholdGen;
    int m_OpeningSize;
    
    bool m_bRecompute;
    vector<cv::BackgroundSubtractorMOG2> m_Subtractors;
};

template<>
class BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame> : public BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, ColorFrame>
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame>& rhs);
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame>& rhs);
    
    void setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<ColorFrame::Ptr> frames);
    vector<ColorFrame::Ptr> getInputFrames() const;
    
    void setShadowsModeling(bool shadowsModeling = true);
    
    void model();
    
    void subtract(vector<cv::Mat>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame> > Ptr;

private:
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, ColorFrame::Ptr frame, cv::Mat& fgMask);
    
    Sequence<ColorFrame>::Ptr m_pSeq;
    vector<ColorFrame::Ptr> m_InputFrames;
    bool m_bShadowsModeling;
};

template<>
class BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame> : public BackgroundSubtractorBase<cv::BackgroundSubtractorMOG2, DepthFrame>
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame>& rhs);
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame>& rhs);
    
    void setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<DepthFrame::Ptr> frames);
    vector<DepthFrame::Ptr> getInputFrames() const;
    
    void model();
    
    void subtract(vector<cv::Mat>& fgMasks);
    
    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame> > Ptr;
    
private:
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, DepthFrame::Ptr frame, cv::Mat& fgMask);
    
    Sequence<DepthFrame>::Ptr m_pSeq;
    vector<DepthFrame::Ptr> m_InputFrames;
};

template<>
class BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame> : public BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorFrame>, public BackgroundSubtractor<cv::BackgroundSubtractorMOG2, DepthFrame>
{
public:
    BackgroundSubtractor();
    BackgroundSubtractor(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>& rhs);
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>& operator=(const BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>& rhs);
    
    void setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<ColorDepthFrame::Ptr> frames);
    
    void setFramesResolution(int xres, int yres);
    
    void setModality(int m);
    int getModality() const;
    
    void setNumOfMixtureComponents(int k);
    void setLearningRate(float rate);
    void setBackgroundRatio(float ratio);
    void setVarThresholdGen(float var);
    void setOpeningSize(int level = 0);
    
    void model();
    void subtract(vector<cv::Mat>& fgMasks);
    
    typedef boost::shared_ptr<BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame> > Ptr;
    
private:
    void _model();
    void subtract(cv::BackgroundSubtractorMOG2& subtractor, ColorDepthFrame::Ptr frame, cv::Mat& fgMask);
    
    int m_XRes;
    int m_YRes;
    
    int m_NumOfSamples;
    
    int m_NumOfMixtureComponents;
    float m_LearningRate;
    float m_BackgroundRatio;
    float m_VarThresholdGen;
    int m_OpeningSize;
    
    bool m_bRecompute;
    vector<cv::BackgroundSubtractorMOG2> m_Subtractors;
    
    int m_Modality;
    
    Sequence<ColorDepthFrame>::Ptr m_pSeq;
    vector<ColorDepthFrame::Ptr> m_InputFrames;
};
