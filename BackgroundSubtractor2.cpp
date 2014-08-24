#include "BackgroundSubtractor2.h"

#include "DepthFrame.h"
#include "ColorFrame.h"

#include "cvxtended.h"

#include <boost/timer.hpp>

template<typename FrameT>
BackgroundSubtractor2Base<FrameT>::BackgroundSubtractor2Base()
: m_NumOfSamples(400), m_NumOfMixtureComponents(4), m_LearningRate(0.02), m_BackgroundRatio(0.999), m_bRecompute(false)
{ }

template<typename FrameT>
BackgroundSubtractor2Base<FrameT>::BackgroundSubtractor2Base(const BackgroundSubtractor2Base<FrameT>& rhs)
{
    *this = rhs;
}

template<typename FrameT>
BackgroundSubtractor2Base<FrameT>& BackgroundSubtractor2Base<FrameT>::operator=(const BackgroundSubtractor2Base<FrameT>& rhs)
{
    if (this != &rhs)
    {
        m_NumOfMixtureComponents = rhs.m_NumOfMixtureComponents;
        m_LearningRate = rhs.m_LearningRate;
        
        m_BackgroundData = rhs.m_BackgroundData;
        m_MOGs = rhs.m_MOGs;
        
        m_bRecompute = rhs.m_bRecompute;
        m_bComputeModels = rhs.m_bComputeModels;
    }
    
    return *this;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setFramesResolution(int xres, int yres)
{
    m_XRes = xres;
    m_YRes = yres;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setNumOfMixtureComponents(int k)
{
    m_bRecompute = (m_NumOfMixtureComponents != k);
    m_NumOfMixtureComponents = k;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setLearningRate(float rate)
{
    m_bRecompute = (m_LearningRate != rate);
    m_LearningRate = rate;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setBackgroundRatio(float ratio)
{
    m_bRecompute = (m_BackgroundRatio != ratio);
    m_BackgroundRatio = ratio;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setVarGenThreshold(float var)
{
    m_bRecompute = (m_VarGenThreshold != var);
    m_VarGenThreshold = var;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setMorphologyLevel(int level)
{
    m_MorphLevel = level;
}

template<typename FrameT>
int BackgroundSubtractor2Base<FrameT>::getNumOfSamples()
{
    return m_NumOfSamples;
}

template<typename FrameT>
int BackgroundSubtractor2Base<FrameT>::getNumOfMixtureComponents()
{
    return m_NumOfMixtureComponents;
}

template<typename FrameT>
float BackgroundSubtractor2Base<FrameT>::getLearningRate()
{
    return m_LearningRate;
}

template<typename FrameT>
float BackgroundSubtractor2Base<FrameT>::getBackgroundRatio()
{
    return m_BackgroundRatio;
}

template<typename FrameT>
float BackgroundSubtractor2Base<FrameT>::getVarGenThreshold()
{
    return m_VarGenThreshold;
}

template<typename FrameT>
int BackgroundSubtractor2Base<FrameT>::getMorphologyLevel()
{
    return m_MorphLevel;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::setComputeModels(bool bComputeModels)
{
    m_bComputeModels = bComputeModels;
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::model(string dirname)
{
    if (m_bRecompute)
    {
        m_MOGs.clear();
        
        m_MOGs.resize(m_BackgroundData.size());
        for (int v = 0; v < m_BackgroundData.size(); v++)
        {
            m_MOGs[v].resize(m_YRes);
            for (int y = 0; y < m_YRes; y++)
            {
                m_MOGs[v][y].resize(m_XRes);
                for (int x = 0; x < m_XRes; x++)
                {
                    m_MOGs[v][y][x] = boost::shared_ptr<cv::EM>(new cv::EM);
                    m_MOGs[v][y][x]->set("nclusters", m_NumOfMixtureComponents);
                }
            }
        }
        
        for (int v = 0; v < m_BackgroundData.size(); v++)
        {
            cout << "Computing/loading model of view " << v << " ..." << endl;
            train( m_BackgroundData[v], dirname + to_string(v), m_MOGs[v]);
        }
        
        m_bRecompute = false;
    }
}

template<typename FrameT>
void BackgroundSubtractor2Base<FrameT>::train(vector<vector<cv::Mat> > data, string dirname, vector<vector<boost::shared_ptr<cv::EM> > >& frameMOGs)
{
    cv::FileStorage fs;
    
    for (int y = 0; y < m_YRes; y++)
    {
        for (int x = 0; x < m_XRes; x++)
        {
            string filepath = dirname + "/" + to_string(y) + "-" + to_string(x) + ".yml";
            if (!m_bComputeModels)
            {
                // Load
                fs.open(filepath, cv::FileStorage::READ);
                frameMOGs[y][x]->read( fs["StatModel.EM"] );
                fs.release();
            }
            else
            {
                // Computation
                cv::Mat data_32F;
                (data[y][x]).convertTo( data_32F, CV_MAKETYPE(data[y][x].depth(), data[y][x].type()) );
                frameMOGs[y][x]->train(data_32F);
                
                // Save
                fs.open(filepath, cv::FileStorage::WRITE);
                if (fs.isOpened())
                {
                    frameMOGs[y][x]->write(fs);
                    fs.release();
                }
            }
        }
    }
}

template<typename FrameT>
bool BackgroundSubtractor2Base<FrameT>::test(boost::shared_ptr<cv::EM> frameMOGs, cv::Mat point)
{
    
}

//

// BackgroundSubtractor2<ColorFrame>
//

BackgroundSubtractor2<ColorFrame>::BackgroundSubtractor2()
: BackgroundSubtractor2Base<ColorFrame>(), m_bShadowsModeling(false)
{
    
}

BackgroundSubtractor2<ColorFrame>::BackgroundSubtractor2(const BackgroundSubtractor2<ColorFrame>& rhs)
: BackgroundSubtractor2Base<ColorFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor2<ColorFrame>& BackgroundSubtractor2<ColorFrame>::operator=(const BackgroundSubtractor2<ColorFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractor2Base<ColorFrame>::operator=(rhs);
        
        m_pSeq = rhs.m_pSeq;
        m_InputFrames = rhs.m_InputFrames;
    }
    
    return *this;
}

void BackgroundSubtractor2<ColorFrame>::setBackgroundSequence(Sequence<ColorFrame>::Ptr pSeq, int n)
{
    m_bRecompute = true;
    m_pSeq = pSeq;
    
    m_BackgroundData.resize(m_pSeq->getNumOfViews());
    for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
    {
        m_BackgroundData[v].resize(m_YRes);
        for (int y = 0; y < m_YRes; y++)
        {
            m_BackgroundData[v][y].resize(m_XRes);
            for (int x = 0; x < m_XRes; x++)
                m_BackgroundData[v][y][x].create(0, 3, CV_8UC1); // channels turned to columns
        }
    }
    
    if (m_bComputeModels)
    {
        int t = 0;
        
        pSeq->restart();
        while (m_pSeq->hasNextFrames() && t++ < n)
        {
            cout << t << endl;
            vector<ColorFrame::Ptr> frames = m_pSeq->nextFrames();
            for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
            {
                cv::Mat mat = frames[v]->get();
            
                for (int y = 0; y < m_YRes; y++) for (int x = 0; x < m_XRes; x++)
                {
                    cv::Vec3b value = mat.at<cv::Vec3b>(y,x);
                    
                    // cv::mat channels do not serve as dimensions, instead channels to columns conversion
                    cv::Mat point (1, 3, CV_8UC1);
                    point.at<uchar>(0,0) = value[0];
                    point.at<uchar>(0,1) = value[1];
                    point.at<uchar>(0,2) = value[2];
                    m_BackgroundData[v][y][x].push_back(point);
                }
            }
        }
    }
}

void BackgroundSubtractor2<ColorFrame>::setInputFrames(vector<ColorFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

vector<ColorFrame::Ptr> BackgroundSubtractor2<ColorFrame>::getInputFrames() const
{
    return m_InputFrames;
}

void BackgroundSubtractor2<ColorFrame>::setShadowsModeling(bool flag)
{
    m_bRecompute = (m_bShadowsModeling != flag);
    m_bShadowsModeling = flag;
}

void BackgroundSubtractor2<ColorFrame>::subtract(int v, ColorFrame& frame)
{
    ColorFrame::Ptr pInputFrame = m_InputFrames[v];
    subtract(m_MOGs[v], pInputFrame, frame);
}

void BackgroundSubtractor2<ColorFrame>::subtract(vector<ColorFrame::Ptr>& colorFgFrames)
{
	for (int v = 0; v < m_InputFrames.size(); v++)
        subtract( v, *(m_InputFrames[v]) );
}

void BackgroundSubtractor2<ColorFrame>::subtract(vector<vector<boost::shared_ptr<cv::EM> > > frameMOGs, ColorFrame::Ptr colorFrame, ColorFrame& colorFgFrame)
{
    cv::Mat image = colorFrame->get();// bgr2hs(colorFrame->get());
    cv::Mat mask (image.rows, image.cols, cv::DataType<uchar>::type, cv::Scalar(0));
    
    for (int y = 0; y < m_YRes; y++)
    {
        for (int x = 0; x < m_XRes; x++)
        {
            cv::Vec3b value = image.at<cv::Vec3b>(y,x);
            
            // cv::mat channels do not serve as dimensions, instead channels to columns conversion
            cv::Mat point (1, 3, CV_8UC1);
            point.at<uchar>(0,0) = value[0];
            point.at<uchar>(0,1) = value[1];
            point.at<uchar>(0,2) = value[2];

            mask.at<uchar>(y,x) = test(frameMOGs[y][x], point);
        }
    }
    
    cvx::open(mask, m_MorphLevel, mask);
    
	colorFgFrame = *colorFrame;
	colorFgFrame.setMask(mask == 255); // no background (0), no shadows (127)
}


//
// BackgroundSubtractor2<DepthFrame>
//

BackgroundSubtractor2<DepthFrame>::BackgroundSubtractor2()
: BackgroundSubtractor2Base<DepthFrame>()
{
    
}

BackgroundSubtractor2<DepthFrame>::BackgroundSubtractor2(const BackgroundSubtractor2<DepthFrame>& rhs)
: BackgroundSubtractor2Base<DepthFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor2<DepthFrame>& BackgroundSubtractor2<DepthFrame>::operator=(const BackgroundSubtractor2<DepthFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractor2Base<DepthFrame>::operator=(rhs);
        
        m_pSeq = rhs.m_pSeq;
        m_InputFrames = rhs.m_InputFrames;
    }
    
    return *this;
}

void BackgroundSubtractor2<DepthFrame>::setBackgroundSequence(Sequence<DepthFrame>::Ptr pSeq, int n)
{
    m_bRecompute = true;
    m_pSeq = pSeq;
    
    m_BackgroundData.resize(m_pSeq->getNumOfViews());
    for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
    {
        m_BackgroundData[v].resize(m_YRes);
        for (int y = 0; y < m_YRes; y++)
        {
            m_BackgroundData[v][y].resize(m_XRes);
            for (int x = 0; x < m_XRes; x++)
                m_BackgroundData[v][y][x].create(0, 1, CV_16UC1);
        }
    }
    
    if (m_bComputeModels)
    {
        int t = 0;
        
        pSeq->restart();
        while (m_pSeq->hasNextFrames() && t++ < n)
        {
            vector<DepthFrame::Ptr> frames = m_pSeq->nextFrames();
            for (int v = 0; v < m_pSeq->getNumOfViews(); v++)
            {
                cv::Mat mat = frames[v]->get();
                for (int y = 0; y < m_YRes; y++) for (int x = 0; x < m_XRes; x++)
                {
                    unsigned short value = mat.at<unsigned short>(y,x);
                    if (value > 0.f) // not a depth error
                        m_BackgroundData[v][y][x].push_back( cv::Mat(1, 1, CV_16UC1, cv::Scalar(value)) );
                }
            }
        }
    }
}

void BackgroundSubtractor2<DepthFrame>::setInputFrames(vector<DepthFrame::Ptr> frames)
{
    m_InputFrames = frames;
}

vector<DepthFrame::Ptr> BackgroundSubtractor2<DepthFrame>::getInputFrames() const
{
    return m_InputFrames;
}

void BackgroundSubtractor2<DepthFrame>::subtract(int v, DepthFrame& frame)
{
    subtract(m_MOGs[v], m_InputFrames[v], frame);
}

void BackgroundSubtractor2<DepthFrame>::subtract(vector<DepthFrame::Ptr>& depthFgFrames)
{
	for (int v = 0; v < m_InputFrames.size(); v++)
        subtract(v, *(depthFgFrames[v]) );
}

void BackgroundSubtractor2<DepthFrame>::subtract(vector<vector<boost::shared_ptr<cv::EM> > > frameMOGs, DepthFrame::Ptr depthFrame, DepthFrame& depthFgFrame)
{
    cv::Mat depthMap;
    depthFrame->get().convertTo(depthMap, CV_32F);
    cv::Mat depthMask;
    
//    test(frameMOGs, 255 * (depthMap / 4096.f), depthMask);
//    // TODO
//    subtractor(255 * (depthMap / 4096.f), depthMask, 0.0000000000000000000000001);
    
    cvx::open(depthMask, m_MorphLevel, depthMask);
    
	depthFgFrame = *depthFrame;
	depthFgFrame.setMask(depthMask == 255);
}


//
// BackgroundSubtractor2<ColorDepthFrame>
//

BackgroundSubtractor2<ColorDepthFrame>::BackgroundSubtractor2()
: BackgroundSubtractor2<ColorFrame>(), BackgroundSubtractor2<DepthFrame>()
{
    
}

BackgroundSubtractor2<ColorDepthFrame>::BackgroundSubtractor2(const BackgroundSubtractor2<ColorDepthFrame>& rhs)
: BackgroundSubtractor2<ColorFrame>(rhs), BackgroundSubtractor2<DepthFrame>(rhs)
{
    *this = rhs;
}

BackgroundSubtractor2<ColorDepthFrame>& BackgroundSubtractor2<ColorDepthFrame>::operator=(const BackgroundSubtractor2<ColorDepthFrame>& rhs)
{
    if (this != &rhs)
    {
        BackgroundSubtractor2<ColorFrame>::operator=(rhs);
        BackgroundSubtractor2<DepthFrame>::operator=(rhs);
        
        m_Modality = rhs.m_Modality;
    }
    
    return *this;
}

void BackgroundSubtractor2<ColorDepthFrame>::setBackgroundSequence(Sequence<ColorDepthFrame>::Ptr pSeq, int n)
{
    Sequence<ColorFrame>::Ptr pColorSeq (new Sequence<ColorFrame>);
    Sequence<DepthFrame>::Ptr pDepthSeq (new Sequence<DepthFrame>);
    
    *pColorSeq = *pSeq;
    *pDepthSeq = *pSeq;
    
    BackgroundSubtractor2<ColorFrame>::setBackgroundSequence(pColorSeq, n);
    BackgroundSubtractor2<DepthFrame>::setBackgroundSequence(pDepthSeq, n);
}

void BackgroundSubtractor2<ColorDepthFrame>::setInputFrames(vector<ColorDepthFrame::Ptr> frames)
{
    vector<ColorFrame::Ptr> colorFrames (frames.size());
    vector<DepthFrame::Ptr> depthFrames (frames.size());
    
    for (int v = 0; v < frames.size(); v++)
    {
        colorFrames[v] = frames[v];
        depthFrames[v] = frames[v];
    }
    
    BackgroundSubtractor2<ColorFrame>::setInputFrames(colorFrames);
    BackgroundSubtractor2<DepthFrame>::setInputFrames(depthFrames);
}

void BackgroundSubtractor2<ColorDepthFrame>::setFramesResolution(int xres, int yres)
{
    BackgroundSubtractor2<ColorFrame>::setFramesResolution(xres, yres);
    BackgroundSubtractor2<DepthFrame>::setFramesResolution(xres, yres);
}

void BackgroundSubtractor2<ColorDepthFrame>::setNumOfMixtureComponents(int k)
{
    BackgroundSubtractor2<ColorFrame>::setNumOfMixtureComponents(k);
    BackgroundSubtractor2<DepthFrame>::setNumOfMixtureComponents(k);
}

void BackgroundSubtractor2<ColorDepthFrame>::setLearningRate(float rate)
{
    BackgroundSubtractor2<ColorFrame>::setLearningRate(rate);
    BackgroundSubtractor2<DepthFrame>::setLearningRate(rate);
}

void BackgroundSubtractor2<ColorDepthFrame>::setBackgroundRatio(float ratio)
{
    BackgroundSubtractor2<ColorFrame>::setBackgroundRatio(ratio);
    BackgroundSubtractor2<DepthFrame>::setBackgroundRatio(ratio);
}

void BackgroundSubtractor2<ColorDepthFrame>::setVarGenThreshold(float var)
{
    BackgroundSubtractor2<ColorFrame>::setVarGenThreshold(var);
    BackgroundSubtractor2<DepthFrame>::setVarGenThreshold(var);
}

void BackgroundSubtractor2<ColorDepthFrame>::setMorphologyLevel(int level)
{
    BackgroundSubtractor2<ColorFrame>::setMorphologyLevel(level);
    BackgroundSubtractor2<DepthFrame>::setMorphologyLevel(level);
}

void BackgroundSubtractor2<ColorDepthFrame>::setModality(int m)
{
    m_Modality = m;
    
    if (m == 1)
        BackgroundSubtractor2<ColorFrame>::setShadowsModeling(true);
    else
        BackgroundSubtractor2<ColorFrame>::setShadowsModeling(false);
}

int BackgroundSubtractor2<ColorDepthFrame>::getModality() const
{
    return m_Modality;
}
                        
void BackgroundSubtractor2<ColorDepthFrame>::setComputeModels(bool bCompute)
{
    BackgroundSubtractor2<ColorFrame>::setComputeModels(bCompute);
    BackgroundSubtractor2<DepthFrame>::setComputeModels(bCompute);
}

void BackgroundSubtractor2<ColorDepthFrame>::model(string path)
{
    if ( (m_Modality == 0) || (m_Modality == 1) )
        BackgroundSubtractor2<ColorFrame>::model(path);
    else
        BackgroundSubtractor2<DepthFrame>::model(path);
}

void BackgroundSubtractor2<ColorDepthFrame>::subtract(vector<ColorDepthFrame::Ptr>& fgFrames)
{
    if (m_Modality == 0 || m_Modality == 1)
    {
        for (int v = 0; v < BackgroundSubtractor2<ColorFrame>::getInputFrames().size(); v++)
            BackgroundSubtractor2<ColorFrame>::subtract( v, *(fgFrames[v]) );
    }
    else
    {
        for (int v = 0; v < BackgroundSubtractor2<DepthFrame>::getInputFrames().size(); v++)
            BackgroundSubtractor2<DepthFrame>::subtract( v, *(fgFrames[v]) );
    }
}

// Template instantiaions

template class BackgroundSubtractor2Base<Frame>;
template class BackgroundSubtractor2Base<ColorFrame>;
template class BackgroundSubtractor2Base<DepthFrame>;

template class BackgroundSubtractor2<Frame>;
template class BackgroundSubtractor2<ColorFrame>;
template class BackgroundSubtractor2<DepthFrame>;