#include "BackgroundSubtractor.h"

#include "DepthFrame.h"
#include "ColorFrame.h"

#include "CvExtraTools.h"

#include <boost/timer.hpp>

int depthType = CV_16UC1;

template<typename FrameT>
BackgroundSubtractorBase<FrameT>::BackgroundSubtractorBase()
    : m_NumOfSamples(400), m_NumOfMixtureComponents(4), m_LearningRate(0.02), m_BackgroundRatio(0.999)
{ }

template<typename FrameT>
BackgroundSubtractorBase<FrameT>::BackgroundSubtractorBase(const BackgroundSubtractorBase<FrameT>& rhs)
{
    *this = rhs;
}

template<typename FrameT>
BackgroundSubtractorBase<FrameT>::~BackgroundSubtractorBase()
{

}

template<typename FrameT>
BackgroundSubtractorBase<FrameT>& BackgroundSubtractorBase<FrameT>::operator=(const BackgroundSubtractorBase<FrameT>& rhs)
{
    if (this != &rhs)
    {
        m_pSeq = rhs.m_pSeq;
        m_NumOfSamples = rhs.m_NumOfSamples;
        m_NumOfMixtureComponents = rhs.m_NumOfMixtureComponents;
        m_LearningRate = rhs.m_LearningRate;
        
        m_Subtractors = rhs.m_Subtractors;
        m_MeanFrames = rhs.m_MeanFrames;
        m_StddevFrames = rhs.m_StddevFrames;

        m_StationaryErrorsFrames = rhs.m_StationaryErrorsFrames;
    }
    
    return *this;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setInputSequence(typename Sequence<FrameT>::Ptr pSeq)
{
    m_pSeq = pSeq;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setNumOfSamples(int n)
{
    m_NumOfSamples = n;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setNumOfMixtureComponents(int k)
{
    m_NumOfMixtureComponents = k;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setLearningRate(float rate)
{
    m_LearningRate = rate;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setBackgroundRatio(float ratio)
{
    m_BackgroundRatio = ratio;
    
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setMorphologyLevel(int level)
{
    m_MorphLevel = level;
}

template<typename FrameT>
int BackgroundSubtractorBase<FrameT>::getNumOfSamples()
{
    return m_NumOfSamples;
}

template<typename FrameT>
int BackgroundSubtractorBase<FrameT>::getNumOfMixtureComponents()
{
    return m_NumOfMixtureComponents;
}

template<typename FrameT>
float BackgroundSubtractorBase<FrameT>::getLearningRate()
{
    return m_LearningRate;
}

template<typename FrameT>
float BackgroundSubtractorBase<FrameT>::getBackgroundRatio()
{
    return m_BackgroundRatio;
}

template<typename FrameT>
int BackgroundSubtractorBase<FrameT>::getMorphologyLevel()
{
    return m_MorphLevel;
}

template<typename FrameT> template<typename T>
void BackgroundSubtractorBase<FrameT>::accumulate(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts)
{
    cv::Mat _src = src.getMat();
    cv::Mat _err = err.getMat();
    cv::Mat& _dst = dst.getMatRef();
    
    if (_dst.empty())
    {
        _dst = cv::Mat(_src.rows, _src.cols, _src.type(), cv::Scalar(0));
        
        if (counts.kind() > 0)
        {
            counts.getMatRef() = cv::Mat(_src.rows, _src.cols, cv::DataType<int>::type, cv::Scalar(0));
        }
    }
    
    for (int i = 0; i < _dst.rows; i++) for (int j = 0; j < _dst.cols; j++)
    {
        if (err.kind() == 0 || _err.empty() || _err.at<unsigned char>(i,j) == 0)
        {
            _dst.at<T>(i,j) += _src.at<T>(i,j);
            if (counts.kind() > 0)
                counts.getMatRef().at<int>(i,j) ++;
        }
    }
}

template<typename FrameT> template<typename T>
void BackgroundSubtractorBase<FrameT>::accumulateSquareDifferences(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts)
{
    cv::Mat _src1 = src1.getMat();
    cv::Mat _src2 = src2.getMat();
    cv::Mat _err = err.getMat();
    cv::Mat& _dst = dst.getMatRef();
    
    if (_dst.empty())
    {
        _dst = cv::Mat(_src1.rows, _src1.cols, _src1.type(), cv::Scalar(0));
        
        if (counts.kind() > 0)
        {
            counts.getMatRef() = cv::Mat(_src1.rows, _src1.cols, cv::DataType<int>::type, cv::Scalar(0));
        }
    }
    
    cv::Mat tmp (_dst.rows, _dst.cols, _dst.type(), cv::Scalar(0));
    for (int i = 0; i < _dst.rows; i++) for (int j = 0; j < _dst.cols; j++)
    {
        T t1 = _src1.at<T>(i,j);
        T t2 = _src2.at<T>(i,j);
        if (err.kind() == 0 || _err.empty() || _err.at<unsigned char>(i,j) == 0)
        {
            tmp.at<T>(i,j) += (t1 - t2);
            if (counts.kind() > 0)
                counts.getMatRef().at<int>(i,j)++;
        }
    }
    
    cv::pow(tmp, 2, tmp);
    cv::add(_dst, tmp, _dst);
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::standardize(cv::InputArray frame, cv::InputArray mean, cv::InputArray stddev, cv::InputArray errors, cv::OutputArray frameStd)
{
    cv::Mat _stddev = stddev.getMat();
    
    if (errors.kind() > 0 && !errors.getMat().empty())
        _stddev.setTo(1, errors);

    cv::Mat _frame, _frameStd;
    frame.getMat().convertTo(_frame, cv::DataType<float>::type);
    _frameStd = (_frame - mean.getMat()) / _stddev;
    
    if (errors.kind() > 0 && !errors.getMat().empty())
        _frameStd.setTo(0, errors); // 0 is the mean
    
    frameStd.getMatRef() = _frameStd;
}

void BackgroundSubtractor<DepthFrame>::model()
{
    m_Subtractors.resize(m_pSeq->getNumOfViews());
    for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
    {
        m_Subtractors[v] = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[v], m_NumOfMixtureComponents, false);
        m_Subtractors[v].set("backgroundRatio", m_BackgroundRatio);
    }
    
    vector<DepthFrame::Ptr> depthFrames;
    
    int t = 1;
    m_pSeq->restart();
	while (t <= m_NumOfSamples && m_pSeq->hasNextFrame())
	{
        depthFrames = m_pSeq->nextFrame();
        
        vector<cv::Mat> depthMaps (m_pSeq->getNumOfViews());
        vector<cv::Mat> depthMasks (depthMaps.size());
        for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
        {
            depthFrames[v]->get().convertTo(depthMaps[v], cv::DataType<float>::type);
            m_Subtractors[v](255.f * (depthMaps[v] / 4096.f), depthMasks[v], 1.f/m_NumOfSamples);
        }
        
        t++;
	}
}

void BackgroundSubtractor<ColorFrame>::model()
{
    m_Subtractors.resize(m_pSeq->getNumOfViews());
    for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
    {
        m_Subtractors[v] = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[v], m_NumOfMixtureComponents, true);
        m_Subtractors[v].set("backgroundRatio", m_BackgroundRatio);
    }
    
    vector<ColorFrame::Ptr> colorFrames;
    
    int t = 1;
    m_pSeq->restart();
	while (t <= m_NumOfSamples && m_pSeq->hasNextFrame())
	{
        colorFrames = m_pSeq->nextFrame();
        
        vector<cv::Mat> hsImages (m_pSeq->getNumOfViews());
        vector<cv::Mat> hsMasks (hsImages.size());
        
        for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
        {
            cv::Mat bgrImage = colorFrames[v]->get();
            hsImages[v] = bgr2hs(bgrImage);
            
            m_Subtractors[v](hsImages[v], hsMasks[v],  (m_LearningRate == 0 || t == 1) ? (1.f/t) : m_LearningRate);
        }
        
        t++;
	}
}

void BackgroundSubtractor<ColorFrame>::subtract(vector<ColorFrame::Ptr> colorFrames, vector<ColorFrame::Ptr>& colorFgFrames)
{
	for (int v = 0; v < colorFrames.size(); v++)
    {
        subtract(m_Subtractors[v], colorFrames[v], m_MeanFrames[v], m_StddevFrames[v], *(colorFgFrames[v]));
    }
}

void BackgroundSubtractor<DepthFrame>::subtract(vector<DepthFrame::Ptr> depthFrames, vector<DepthFrame::Ptr>& depthFgFrames)
{
	for (int v = 0; v < depthFrames.size(); v++)
    {
        subtract(m_Subtractors[v], depthFrames[v], m_MeanFrames[v], m_StddevFrames[v], m_StationaryErrorsFrames[v], *(depthFgFrames[v]));
    }
}

void BackgroundSubtractor<ColorFrame>::subtract(cv::BackgroundSubtractorMOG2& subtractor, ColorFrame::Ptr colorFrame, cv::Mat mean, cv::Mat stddev, ColorFrame& colorFgFrame)
{
    cv::Mat hsMat = bgr2hs(colorFrame->get());
    cv::Mat hsMask;

    subtractor(hsMat, hsMask, 0.0000000000000000000000001);
    
    cvx::open(hsMask, m_MorphLevel, hsMask);
    
	colorFgFrame = *colorFrame;
	colorFgFrame.setMask(hsMask == 255); // no background (0), no shadows (127)
}

void BackgroundSubtractor<DepthFrame>::subtract(cv::BackgroundSubtractorMOG2& subtractor, DepthFrame::Ptr depthFrame, cv::Mat mean, cv::Mat stddev, cv::Mat errors, DepthFrame& depthFgFrame)
{
    cv::Mat depthMap;
    depthFrame->get().convertTo(depthMap, CV_32F);
    cv::Mat depthMask;
    
    subtractor(255 * (depthMap / 4096.f), depthMask, 0.0000000000000000000000001);
    
    cvx::open(depthMask, m_MorphLevel, depthMask);
    
	depthFgFrame = *depthFrame;
	depthFgFrame.setMask(depthMask == 255);
}


//
// BackgroundSubtractor<ColorFrame>
//
BackgroundSubtractor<ColorFrame>::BackgroundSubtractor()
    : BackgroundSubtractorBase<ColorFrame>()
{
    
}

//
// BackgroundSubtractor<DepthFrame>
//
BackgroundSubtractor<DepthFrame>::BackgroundSubtractor()
    : BackgroundSubtractorBase<DepthFrame>()
{
    
}

// Template instantiaions

template class BackgroundSubtractorBase<Frame>;
template class BackgroundSubtractorBase<ColorFrame>;
template class BackgroundSubtractorBase<DepthFrame>;

template class BackgroundSubtractor<Frame>;
template class BackgroundSubtractor<ColorFrame>;
template class BackgroundSubtractor<DepthFrame>;

template void BackgroundSubtractorBase<Frame>::accumulate<cv::Vec2f>(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);
template void BackgroundSubtractorBase<ColorFrame>::accumulate<float>(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);
template void BackgroundSubtractorBase<DepthFrame>::accumulate<cv::Vec3f>(cv::InputArray src, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);

template void BackgroundSubtractorBase<Frame>::accumulateSquareDifferences<float>(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);
template void BackgroundSubtractorBase<ColorFrame>::accumulateSquareDifferences<cv::Vec2f>(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);
template void BackgroundSubtractorBase<DepthFrame>::accumulateSquareDifferences<cv::Vec3f>(cv::InputArray src1, cv::InputArray src2, cv::InputArray err, cv::OutputArray dst, cv::OutputArray counts);