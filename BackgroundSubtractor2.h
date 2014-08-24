#pragma once

#include "DepthFrame.h"
#include "KinectReader.h"
#include "Sequence.h"

#include <opencv2/opencv.hpp>

template<typename FrameT>
class BackgroundSubtractor2Base
{
public:
    BackgroundSubtractor2Base();
    BackgroundSubtractor2Base(const BackgroundSubtractor2Base<FrameT>& rhs);
    
    BackgroundSubtractor2Base& operator=(const BackgroundSubtractor2Base<FrameT>& rhs);
    
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
    
    void setComputeModels(bool bCompute = true);
    
    void model(string dirname = "");
    void train(vector<vector<cv::Mat> > data, string path, vector<vector<boost::shared_ptr<cv::EM> > >& frameMOGs);
    bool test(boost::shared_ptr<cv::EM> frameMOGs, cv::Mat point);
    
    virtual void setInputFrames(vector<typename FrameT::Ptr> frames) = 0;
    
    
    // Measure overlaps of the sequence views
    //    template<typename T>
    //    bool measureOverlap(typename Sequence<T>::Ptr pValSeq, vector<float>& seqOverlaps);
    
    typedef boost::shared_ptr<BackgroundSubtractor2Base<FrameT> > Ptr;
    
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
    
    // #views * (#rows x #columns)
    vector <vector <vector<cv::Mat> > > m_BackgroundData;
    vector<vector<vector<boost::shared_ptr<cv::EM> > > > m_MOGs;
    
    bool m_bRecompute;
    
    bool m_bComputeModels;
};

template<typename FrameT>
class BackgroundSubtractor2 : public BackgroundSubtractor2Base<FrameT>
{
public:
    typedef boost::shared_ptr<BackgroundSubtractor2<FrameT> > Ptr;
};

template<>
class BackgroundSubtractor2<ColorFrame> : public BackgroundSubtractor2Base<ColorFrame>
{
public:
    BackgroundSubtractor2();
    BackgroundSubtractor2(const BackgroundSubtractor2<ColorFrame>& rhs);
    
    BackgroundSubtractor2<ColorFrame>& operator=(const BackgroundSubtractor2<ColorFrame>& rhs);
    
    void setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<ColorFrame::Ptr> frames);
    vector<ColorFrame::Ptr> getInputFrames() const;
    
    void setShadowsModeling(bool shadowsModeling = true);
    
    void subtract(vector<ColorFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor2<ColorFrame> > Ptr;
    
protected:
    void subtract(int v, ColorFrame& fgFrame);
    
private:
    void subtract(vector<vector<boost::shared_ptr<cv::EM> > > frameMOGs, ColorFrame::Ptr frame, ColorFrame& fgFrame);
    
    Sequence<ColorFrame>::Ptr m_pSeq;
    vector<ColorFrame::Ptr> m_InputFrames;
    bool m_bShadowsModeling;
};

template<>
class BackgroundSubtractor2<DepthFrame> : public BackgroundSubtractor2Base<DepthFrame>
{
public:
    BackgroundSubtractor2();
    BackgroundSubtractor2(const BackgroundSubtractor2<DepthFrame>& rhs);
    
    BackgroundSubtractor2<DepthFrame>& operator=(const BackgroundSubtractor2<DepthFrame>& rhs);
    
    void setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n = 0);
    void setInputFrames(vector<DepthFrame::Ptr> frames);
    vector<DepthFrame::Ptr> getInputFrames() const;
    
    void subtract(vector<DepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor2<DepthFrame> > Ptr;
    
protected:
    void subtract(int v, DepthFrame& fgFrame);
    
private:
    void subtract(vector<vector<boost::shared_ptr<cv::EM> > > frameMOGs, DepthFrame::Ptr frame, DepthFrame& fgFrame);
    
    Sequence<DepthFrame>::Ptr m_pSeq;
    vector<DepthFrame::Ptr> m_InputFrames;
};

template<>
class BackgroundSubtractor2<ColorDepthFrame> : public BackgroundSubtractor2<ColorFrame>, public BackgroundSubtractor2<DepthFrame>
{
public:
    BackgroundSubtractor2();
    BackgroundSubtractor2(const BackgroundSubtractor2<ColorDepthFrame>& rhs);
    
    BackgroundSubtractor2<ColorDepthFrame>& operator=(const BackgroundSubtractor2<ColorDepthFrame>& rhs);
    
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
    
    void setComputeModels(bool bCompute = true);
    
    void model(string dirname = "");
    void subtract(vector<ColorDepthFrame::Ptr>& fgFrames);
    
    typedef boost::shared_ptr<BackgroundSubtractor2<ColorDepthFrame> > Ptr;
    
private:
    
    //
    // Members
    //
    
    int m_Modality;
};
