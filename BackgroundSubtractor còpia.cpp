#include "BackgroundSubtractor.h"

#include "DepthFrame.h"
#include "ColorFrame.h"

#include "cvxtended.h"

#include <boost/timer.hpp>

int depthType = CV_16UC1;

template<typename FrameT>
BackgroundSubtractorBase<FrameT>::BackgroundSubtractorBase()
    : m_NumOfSamples(400), m_NumOfMixtureComponents(4), m_LearningRate(0.02), m_BackgroundRatio(0.999), m_bRecompute(false)
{ }

template<typename FrameT>
BackgroundSubtractorBase<FrameT>::BackgroundSubtractorBase(const BackgroundSubtractorBase<FrameT>& rhs)
{
    *this = rhs;
}

template<typename FrameT>
BackgroundSubtractorBase<FrameT>& BackgroundSubtractorBase<FrameT>::operator=(const BackgroundSubtractorBase<FrameT>& rhs)
{
    if (this != &rhs)
    {
        m_NumOfSamples = rhs.m_NumOfSamples;
        m_NumOfMixtureComponents = rhs.m_NumOfMixtureComponents;
        m_LearningRate = rhs.m_LearningRate;
        
        m_bRecompute = rhs.m_bRecompute;
        
        m_Subtractors = rhs.m_Subtractors;
        m_MeanFrames = rhs.m_MeanFrames;
        m_StddevFrames = rhs.m_StddevFrames;

        m_StationaryErrorsFrames = rhs.m_StationaryErrorsFrames;
    }
    
    return *this;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setFramesResolution(int xres, int yres)
{
    m_XRes = xres;
    m_YRes = yres;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setNumOfMixtureComponents(int k)
{
    m_bRecompute = (m_NumOfMixtureComponents != k);
    m_NumOfMixtureComponents = k;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setLearningRate(float rate)
{
    m_bRecompute = (m_LearningRate != rate);
    m_LearningRate = rate;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setBackgroundRatio(float ratio)
{
    m_bRecompute = (m_BackgroundRatio != ratio);
    m_BackgroundRatio = ratio;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setVarGenThreshold(float var)
{
    m_bRecompute = (m_VarGenThreshold != var);
    m_VarGenThreshold = var;
}

template<typename FrameT>
void BackgroundSubtractorBase<FrameT>::setMorphologyLevel(int level)
{
    m_MorphLevel = level;
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
float BackgroundSubtractorBase<FrameT>::getVarGenThreshold()
{
    return m_VarGenThreshold;
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


//
// BackgroundSubtractor<ColorFrame>
//

BackgroundSubtractor<ColorFrame>::BackgroundSubtractor()
: BackgroundSubtractorBase<ColorFrame>(), m_bShadowsModeling(false)
{
    
}

BackgroundSubtractor<ColorFrame>::BackgroundSubtractor(const BackgroundSubtractor<ColorFrame>& rhs)
: BackgroundSubtractorBase<ColorFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor<ColorFrame>& BackgroundSubtractor<ColorFrame>::operator=(const BackgroundSubtractor<ColorFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractorBase<ColorFrame>::operator=(rhs);
        
        m_pSeq = rhs.m_pSeq;
        m_InputFrames = rhs.m_InputFrames;
    }
    
    return *this;
}

void BackgroundSubtractor<ColorFrame>::setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n)
{
    m_bRecompute = true;
    m_pSeq = pSeq;
    m_NumOfSamples = n;
}

void BackgroundSubtractor<ColorFrame>::setInputFrames(vector<ColorFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

vector<ColorFrame::Ptr> BackgroundSubtractor<ColorFrame>::getInputFrames() const
{
    return m_InputFrames;
}

void BackgroundSubtractor<ColorFrame>::setShadowsModeling(bool flag)
{
    m_bRecompute = (m_bShadowsModeling != flag);
    m_bShadowsModeling = flag;
}

void BackgroundSubtractor<ColorFrame>::model()
{
    if (m_bRecompute)
    {
        m_Subtractors.resize(m_pSeq->getNumOfViews());
        for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
        {
//            m_Subtractors[v] = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[v], m_NumOfMixtureComponents, m_bShadowsModeling);
            
            m_Subtractors[v] = cv::BackgroundSubtractorGMG();
            m_Subtractors[v].initialize(cv::Size(m_XRes, m_YRes), 0, 255);
            m_Subtractors[v].numInitializationFrames = m_NumOfSamples;
            m_Subtractors[v].quantizationLevels = m_NumOfMixtureComponents;
            m_Subtractors[v].decisionThreshold = m_VarGenThreshold;
//            m_Subtractors[v].createBackgroundSubtractorGMG(m_pSeq->getNumOfFrames()[v]);
//            m_Subtractors[v].set("backgroundRatio", m_BackgroundRatio);
//            m_Subtractors[v].set("varThresholdGen", m_VarGenThreshold);
        }
        
        vector<ColorFrame::Ptr> colorFrames;
        
        int t = 1;
        m_pSeq->restart();
        while (/*t <= m_NumOfSamples &&*/ m_pSeq->hasNextFrames())
        {
            colorFrames = m_pSeq->nextFrames();
            
            vector<cv::Mat> images (m_pSeq->getNumOfViews());
            vector<cv::Mat> masks (images.size());
            
            for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
            {
                cv::Mat bgrImage = colorFrames[v]->get();
                images[v] = bgrImage;//bgr2hs(bgrImage);
                
                m_Subtractors[v].learningRate = (m_LearningRate == 0 || t == 1) ? (1.f/t) : m_LearningRate;
                m_Subtractors[v](images[v], masks[v]);
            }
            
            t++;
        }
    
        for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
            m_Subtractors[v].updateBackgroundModel = false;
        m_bRecompute = false;
    }
}

void BackgroundSubtractor<ColorFrame>::subtract(int v, ColorFrame& frame)
{
    cv::BackgroundSubtractorGMG& subtractor = m_Subtractors[v];
    ColorFrame::Ptr pInputFrame = m_InputFrames[v];
    subtract(subtractor, pInputFrame, frame);
}

void BackgroundSubtractor<ColorFrame>::subtract(vector<ColorFrame::Ptr>& colorFgFrames)
{
	for (int v = 0; v < m_InputFrames.size(); v++)
        subtract( v, *(m_InputFrames[v]) );
}

void BackgroundSubtractor<ColorFrame>::subtract(cv::BackgroundSubtractorGMG& subtractor, ColorFrame::Ptr colorFrame, ColorFrame& colorFgFrame)
{
    cv::Mat image = colorFrame->get();// bgr2hs(colorFrame->get());
    cv::Mat mask;
    
    subtractor(image, mask);//, 0.0000000000000000000000001);
    
    cvx::open(mask, m_MorphLevel, mask);
    
	colorFgFrame = *colorFrame;
	colorFgFrame.setMask(mask == 255); // no background (0), no shadows (127)
}


//
// BackgroundSubtractor<DepthFrame>
//

BackgroundSubtractor<DepthFrame>::BackgroundSubtractor()
: BackgroundSubtractorBase<DepthFrame>()
{
    
}

BackgroundSubtractor<DepthFrame>::BackgroundSubtractor(const BackgroundSubtractor<DepthFrame>& rhs)
: BackgroundSubtractorBase<DepthFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor<DepthFrame>& BackgroundSubtractor<DepthFrame>::operator=(const BackgroundSubtractor<DepthFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractorBase<DepthFrame>::operator=(rhs);
        
        m_pSeq = rhs.m_pSeq;
        m_InputFrames = rhs.m_InputFrames;
    }
    
    return *this;
}

void BackgroundSubtractor<DepthFrame>::setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n)
{
    m_bRecompute = true;
    m_pSeq = pSeq;
    m_NumOfSamples = n;
}

void BackgroundSubtractor<DepthFrame>::setInputFrames(vector<DepthFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

vector<DepthFrame::Ptr> BackgroundSubtractor<DepthFrame>::getInputFrames() const
{
    return m_InputFrames;
}

void BackgroundSubtractor<DepthFrame>::model()
{
    if (m_bRecompute)
    {
        m_Subtractors.resize(m_pSeq->getNumOfViews());
        for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
        {
            //            m_Subtractors[v] = cv::BackgroundSubtractorMOG2(m_pSeq->getNumOfFrames()[v], m_NumOfMixtureComponents, m_bShadowsModeling);
            
//            cv::BackgroundSubtractorGMG gmg;
            m_Subtractors[v] = cv::BackgroundSubtractorGMG();
            m_Subtractors[v].initialize(cv::Size(m_XRes, m_YRes), 1, 4096);
            m_Subtractors[v].numInitializationFrames = m_NumOfSamples;
            m_Subtractors[v].quantizationLevels = m_NumOfMixtureComponents;
            m_Subtractors[v].decisionThreshold = m_VarGenThreshold;
//            m_Subtractors[v] createBackgroundSubtractorGMG(m_pSeq->getNumOfFrames()[v]);
            //            m_Subtractors[v].set("backgroundRatio", m_BackgroundRatio);
            //            m_Subtractors[v].set("varThresholdGen", m_VarGenThreshold);
        }
        
        vector<DepthFrame::Ptr> depthFrames;
        
        int t = 1;
        m_pSeq->restart();
        while (/*t <= m_NumOfSamples &&*/ m_pSeq->hasNextFrames())
        {
            depthFrames = m_pSeq->nextFrames();
            
            vector<cv::Mat> depthMaps (m_pSeq->getNumOfViews());
            vector<cv::Mat> depthMasks (depthMaps.size());
            for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
            {
                depthFrames[v]->get().convertTo(depthMaps[v], cv::DataType<float>::type);
                m_Subtractors[v](255.f * (depthMaps[v] / 4096.f), depthMasks[v], (m_LearningRate == 0 || t == 1) ? (1.f/t) : m_LearningRate);
            }
            
            t++;
        }
        
        m_bRecompute = false;
    }
}

void BackgroundSubtractor<DepthFrame>::subtract(int v, DepthFrame& frame)
{
    subtract(m_Subtractors[v], m_InputFrames[v], frame);
}

void BackgroundSubtractor<DepthFrame>::subtract(vector<DepthFrame::Ptr>& depthFgFrames)
{
	for (int v = 0; v < m_InputFrames.size(); v++)
        subtract(v, *(depthFgFrames[v]) );
}

void BackgroundSubtractor<DepthFrame>::subtract(cv::BackgroundSubtractorGMG& subtractor, DepthFrame::Ptr depthFrame, DepthFrame& depthFgFrame)
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
// BackgroundSubtractor<ColorDepthFrame>
//

BackgroundSubtractor<ColorDepthFrame>::BackgroundSubtractor()
: BackgroundSubtractor<ColorFrame>(), BackgroundSubtractor<DepthFrame>()
{
    
}

BackgroundSubtractor<ColorDepthFrame>::BackgroundSubtractor(const BackgroundSubtractor<ColorDepthFrame>& rhs)
: BackgroundSubtractor<ColorFrame>(rhs), BackgroundSubtractor<DepthFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor<ColorDepthFrame>& BackgroundSubtractor<ColorDepthFrame>::operator=(const BackgroundSubtractor<ColorDepthFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractor<ColorFrame>::operator=(rhs);
        BackgroundSubtractor<DepthFrame>::operator=(rhs);
        
        m_Modality = rhs.m_Modality;
    }
    
    return *this;
}

void BackgroundSubtractor<ColorDepthFrame>::setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pSeq, int n)
{
    Sequence<ColorFrame>::Ptr pColorSeq (new Sequence<ColorFrame>);
    Sequence<DepthFrame>::Ptr pDepthSeq (new Sequence<DepthFrame>);
    
    *pColorSeq = *pSeq;
    *pDepthSeq = *pSeq;
    
    BackgroundSubtractor<ColorFrame>::setBackgroundSequence(pColorSeq, n);
    BackgroundSubtractor<DepthFrame>::setBackgroundSequence(pDepthSeq, n);
}

void BackgroundSubtractor<ColorDepthFrame>::setInputFrames(vector<ColorDepthFrame::Ptr> frames)
{
    vector<ColorFrame::Ptr> colorFrames (frames.size());
    vector<DepthFrame::Ptr> depthFrames (frames.size());
    
    for (int v = 0; v < frames.size(); v++)
    {
        colorFrames[v] = frames[v];
        depthFrames[v] = frames[v];
    }
    
    BackgroundSubtractor<ColorFrame>::setInputFrames(colorFrames);
    BackgroundSubtractor<DepthFrame>::setInputFrames(depthFrames);
}

void BackgroundSubtractor<ColorDepthFrame>::setFramesResolution(int xres, int yres)
{
    BackgroundSubtractor<ColorFrame>::setFramesResolution(xres, yres);
    BackgroundSubtractor<DepthFrame>::setFramesResolution(xres, yres);
}

void BackgroundSubtractor<ColorDepthFrame>::setNumOfMixtureComponents(int k)
{
    BackgroundSubtractor<ColorFrame>::setNumOfMixtureComponents(k);
    BackgroundSubtractor<DepthFrame>::setNumOfMixtureComponents(k);
}

void BackgroundSubtractor<ColorDepthFrame>::setLearningRate(float rate)
{
    BackgroundSubtractor<ColorFrame>::setLearningRate(rate);
    BackgroundSubtractor<DepthFrame>::setLearningRate(rate);
}

void BackgroundSubtractor<ColorDepthFrame>::setBackgroundRatio(float ratio)
{
    BackgroundSubtractor<ColorFrame>::setBackgroundRatio(ratio);
    BackgroundSubtractor<DepthFrame>::setBackgroundRatio(ratio);
}

void BackgroundSubtractor<ColorDepthFrame>::setVarGenThreshold(float var)
{
    BackgroundSubtractor<ColorFrame>::setVarGenThreshold(var);
    BackgroundSubtractor<DepthFrame>::setVarGenThreshold(var);
}

void BackgroundSubtractor<ColorDepthFrame>::setMorphologyLevel(int level)
{
    BackgroundSubtractor<ColorFrame>::setMorphologyLevel(level);
    BackgroundSubtractor<DepthFrame>::setMorphologyLevel(level);
}

void BackgroundSubtractor<ColorDepthFrame>::setModality(int m)
{
    m_Modality = m;
    
    if (m == 1)
        BackgroundSubtractor<ColorFrame>::setShadowsModeling(true);
    else
        BackgroundSubtractor<ColorFrame>::setShadowsModeling(false);
}

int BackgroundSubtractor<ColorDepthFrame>::getModality() const
{
    return m_Modality;
}

void BackgroundSubtractor<ColorDepthFrame>::model()
{
    if ( (m_Modality == 0) || (m_Modality == 1) )
        BackgroundSubtractor<ColorFrame>::model();
    else
        BackgroundSubtractor<DepthFrame>::model();
}

void BackgroundSubtractor<ColorDepthFrame>::subtract(vector<ColorDepthFrame::Ptr>& fgFrames)
{
    if (m_Modality == 0 || m_Modality == 1)
    {
        for (int v = 0; v < BackgroundSubtractor<ColorFrame>::getInputFrames().size(); v++)
            BackgroundSubtractor<ColorFrame>::subtract( v, *(fgFrames[v]) );
    }
    else
    {
        for (int v = 0; v < BackgroundSubtractor<DepthFrame>::getInputFrames().size(); v++)
            BackgroundSubtractor<DepthFrame>::subtract( v, *(fgFrames[v]) );
    }
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