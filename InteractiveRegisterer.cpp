#include "InteractiveRegisterer.h"

#include <opencv2/core/eigen.hpp>
#include "conversion.h"

InteractiveRegisterer::InteractiveRegisterer()
: m_bMark(false)
{
}

InteractiveRegisterer::InteractiveRegisterer(const InteractiveRegisterer& rhs)
{
    *this = rhs;
}

InteractiveRegisterer& InteractiveRegisterer::operator=(const InteractiveRegisterer& rhs)
{
    if (this != &rhs)
    {
        m_NumOfPoints = rhs.m_NumOfPoints;

        m_WndHeight = rhs.m_WndHeight;
        m_WndWidth = rhs.m_WndWidth;
        m_Vp = rhs.m_Vp;
        m_Hp = rhs.m_Hp;
        
        m_pClouds = rhs.m_pClouds;
        m_pMarkers = rhs.m_pMarkers;

        m_Pendents = rhs.m_Pendents;
    }
    
    return *this;
}

/** \brief Set the number of correspondences
 */
void InteractiveRegisterer::setNumPoints(int numOfPoints)
{
	m_NumOfPoints = numOfPoints;
}

/** \brief Set the frames used to establish the correspondences
 */
void InteractiveRegisterer::setInputFrames(vector<ColorDepthFrame::Ptr> pFrames)
{
    m_pFrames = pFrames;
    
    // Init some cloud structures with empty clouds
    
    for (int v = 0; v < m_pFrames.size(); v++)
    {
        // Original untransformed clouds
        ColorPointCloudPtr pColorCloud (new ColorPointCloud);
        m_pFrames[v]->getColoredPointCloud(*pColorCloud);
        m_pClouds.push_back(pColorCloud);
        
        // Marker positions in the views
        PointCloudPtr pMakers (new PointCloud);
        pMakers->width = 0; // we will increment in width
        pMakers->height = 1;
        pMakers->resize(0);
        m_pMarkers.push_back(pMakers);
    }
    
    m_Pendents = pFrames.size();
    
    // Transformation
    m_Transformations.resize( pFrames.size() );
    m_ITransformations.resize( pFrames.size() );
}

/** \brief Set the distribution of the viewports in the visualizer
 * \param wndHeight window height (visor)
 * \param wndWidth window width (visor)
 * \param vp number of vertical ports (visor)
 * \param hp number of horizontal ports (visor)
 * \param camDist camera dist to (0,0,0) (visor)
 * \param markerRadius marker sphere radius (visor)
 */
void InteractiveRegisterer::setVisualizerParameters(int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius)
{
    m_WndHeight = wndHeight;
    m_WndWidth = wndWidth;
    m_Vp = vp;
    m_Hp = hp;
    m_CameraDistance = camDist;
    m_MarkerRadius = markerRadius;
}

/** \brief Manually interact with a visualizer to place the markers that serve as correspondences
 */
void InteractiveRegisterer::setCorrespondencesManually()
{
    // Manual interaction with visualizer
    
    m_pViz = VisualizerPtr(new Visualizer);
    m_pViz->registerMouseCallback (&InteractiveRegisterer::mouseCallback, *this);
    m_pViz->registerKeyboardCallback(&InteractiveRegisterer::keyboardCallback, *this);
    m_pViz->registerPointPickingCallback (&InteractiveRegisterer::ppCallback, *this);
    
    m_pViz->setSize(m_WndWidth * m_Hp, m_WndHeight * m_Vp);
    
    float vside = 1.f / m_Vp; // vertical viewport side size
    float hside = 1.f / m_Hp; // horizontal viewport side size
    for (int v = 0; v < m_pFrames.size(); v++)
    {
        unsigned int x = v % m_Hp;
        unsigned int y = v / m_Hp;
        int vid;
        m_pViz->createViewPort(hside * x, vside * y, hside * (x+1), vside * (y+1), vid);
        m_VIDs.push_back(vid);
        
        m_pViz->addCoordinateSystem(0.1, 0, 0, 0, "cs" + to_string(v), vid);
        m_pViz->addPointCloud(m_pClouds[v], "cloud" + to_string(v), vid);
        
        setDefaultCamera(m_pViz, vid);
    }
    
    while (m_Pendents > 0)
        m_pViz->spinOnce(100);
}

void InteractiveRegisterer::saveCorrespondences(string filename, string extension)
{
    pcl::PCDWriter writer;
    
    for (int v = 0; v < m_pMarkers.size(); v++)
    {
        writer.write(filename + "-" + to_string(v) + "." + extension, *(m_pMarkers[v]));
    }
}

void InteractiveRegisterer::loadCorrespondences(string filename, string extension)
{
    pcl::PCDReader reader;
    
    m_pMarkers.resize(m_pFrames.size());
    for (int v = 0; v < m_pFrames.size(); v++)
    {
        PointCloudPtr pMarkers (new PointCloud);
        reader.read(filename + "-" + to_string(v) + "." + extension, *pMarkers);
        m_pMarkers[v] = pMarkers;
    }
}

/** \brief Estimate the orthogonal transformations in the n-1 views respect to the 0-th view
 */
void InteractiveRegisterer::computeTransformations()
{
    for (int v = 0; v < m_pFrames.size(); v++)
    {
        getTransformation(m_pMarkers[v], m_pMarkers[0], m_Transformations[v]); // v (src) to 0 (tgt)
        m_ITransformations[v] = m_Transformations[v].inverse();
    }
}

/** \brief Registers all the frames to the 0-th frame and return the corresponding registered clouds
 *  \param pUnregFrames A list of unregistered clouds
 *  \param pRegClouds A list of registered clouds
 */
void InteractiveRegisterer::registrate(vector<ColorDepthFrame::Ptr> pUnregFrames, vector<PointCloudPtr>& pRegClouds)
{
    vector<PointCloudPtr> pUnregClouds;

    for (int v = 0; v < pUnregFrames.size(); v++)
    {
        PointCloudPtr pCloud (new PointCloud);
        pUnregFrames[v]->getPointCloud(*pCloud);
        pUnregClouds.push_back(pCloud);
    }
    
    registrate(pUnregClouds, pRegClouds);
}
    
/** \brief Registers all the clouds to the 0-th cloud and return the registered clouds
 *  \param pUnregClouds A list of unregistered clouds
 *  \param pRegClouds A list of registered clouds
 */
void InteractiveRegisterer::registrate(vector<PointCloudPtr> pUnregClouds, vector<PointCloudPtr>& pRegClouds)
{
    pRegClouds.resize(pUnregClouds.size());
    
    for (int v = 0; v < pUnregClouds.size(); v++)
    {
        pRegClouds[v] = PointCloudPtr(new PointCloud);
//        if (v == 0)
//        {
//            translate(pUnregClouds[v], m_pMarkers[v]->points[0], *(pRegClouds[v]));
//        }
//        else
//        {
//            PointCloudPtr pCtrCloud (new PointCloud);
//            translate(pUnregClouds[v], m_pMarkers[v]->points[0], *pCtrCloud);
//            pcl::transformPointCloud(*pCtrCloud, *(pRegClouds[v]), m_Transformations[v]);
//        }
        pcl::transformPointCloud(*(pUnregClouds[v]), *(pRegClouds[v]), m_Transformations[v]);
    }
}

/** \brief Compute the orthogonal tranformation of a set of points to another (from src to target)
 * \param pSrcMarkersCloud Source set of markers to register to target
 * \param pTgtMarkersCloud Target set of markers
 * \param T The 4x4 orthogonal transformation matrix
 */
void InteractiveRegisterer::getTransformation(const PointCloudPtr pSrcMarkersCloud, const PointCloudPtr pTgtMarkersCloud, Eigen::Matrix4f& T)
{
    Eigen::Vector4f centroidSrc, centroidTgt;
    pcl::compute3DCentroid(*pSrcMarkersCloud, centroidSrc);
    pcl::compute3DCentroid(*pTgtMarkersCloud, centroidTgt);
    
    // Translation
    
    PointCloudPtr pSrcCenteredMarkersCloud (new PointCloud);
    PointCloudPtr pTgtCenteredMarkersCloud (new PointCloud);
    
    Eigen::Matrix4f A, B;
    
    A <<
    1, 0, 0,  -centroidSrc.x(),
    0, 1, 0,  -centroidSrc.y(),
    0, 0, 1,  -centroidSrc.z(),
    0, 0, 0,  1;
    
    B <<
    1, 0, 0,  -centroidTgt.x(),
    0, 1, 0,  -centroidTgt.y(),
    0, 0, 1,  -centroidTgt.z(),
    0, 0, 0,  1;

    pcl::transformPointCloud(*pSrcMarkersCloud, *pSrcCenteredMarkersCloud, A);
    pcl::transformPointCloud(*pTgtMarkersCloud, *pTgtCenteredMarkersCloud, B);
    //translate(pSrcMarkersCloud, centroidSrc, *pSrcCenteredMarkersCloud);
//    translate(pTgtMarkersCloud, centroidTgt, *pTgtCenteredMarkersCloud);
    
    // To opencv
        
    int n = 0;
        
    if (pSrcMarkersCloud->width < pTgtMarkersCloud->width)
        n = pSrcMarkersCloud->width;
    else
        n = pTgtMarkersCloud->width;
        
    if (n < 3)
        return; // error
        
    // Fill the matrices
    
    cv::Mat srcMat (3, n, CV_32F);
    cv::Mat tgtMat (3, n, CV_32F);
                
    for (int i = 0; i < n; i++)
    {
        srcMat.at<float>(0,i) = pSrcCenteredMarkersCloud->points[i].x;
        srcMat.at<float>(1,i) = pSrcCenteredMarkersCloud->points[i].y;
        srcMat.at<float>(2,i) = pSrcCenteredMarkersCloud->points[i].z;

        tgtMat.at<float>(0,i) = pTgtCenteredMarkersCloud->points[i].x;
        tgtMat.at<float>(1,i) = pTgtCenteredMarkersCloud->points[i].y;
        tgtMat.at<float>(2,i) = pTgtCenteredMarkersCloud->points[i].z;
    }
        
    cv::Mat h = cv::Mat::zeros(3, 3, CV_32F);
        
    for (int i = 0; i < n; i++)
    {
        cv::Mat aux;
        cv::transpose(tgtMat.col(i), aux);
            
        h = h + (srcMat.col(i) * aux);
            
        aux.release();
    }
        
//    cout << h << endl;
    
    cv::SVD svd;
    cv::Mat s, u, vt;
    svd.compute(h, s, u, vt);
        
    cv::Mat v, ut;
    cv::transpose(vt, v);
    cv::transpose(u, ut);
    vt.release();
        
//    cout << v << endl;
//    cout << (cv::determinant(v) < 0) << endl;
    if (cv::determinant(v) < 0)
    {
        v.col(2) = v.col(2) * (-1);
    }
//    cout << v << endl;
//    cout << (cv::determinant(v) < 0) << endl;
    
    cv::Mat r;
    r = v * ut;
        
//    cout << r << endl;
//    cout << (cv::determinant(r) < 0) << endl;
    if (cv::determinant(r) < 0)
    {
        r.col(2) = r.col(2) * (-1);
    }
//    cout << r << endl;
//    cout << (cv::determinant(r) < 0) << endl;

    Eigen::Matrix4f R;
    R <<
        r.at<float>(0,0), r.at<float>(0,1), r.at<float>(0,2),  0,
        r.at<float>(1,0), r.at<float>(1,1), r.at<float>(1,2),  0,
        r.at<float>(2,0), r.at<float>(2,1), r.at<float>(2,2),  0,
                       0,                0,                0,  1;
        
    cout << R << endl;
    
    // Translate the src to origin (A), rotate (R) to align src to tgt, and perform the inverse translation of target (B^-1).
    T = B.inverse() * R * A;
}

/** \brief Callback function to deal with keyboard presses in visualizer
 */
void InteractiveRegisterer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* ptr)
{
    if (event.keyDown() && event.getKeySym() == "s")
        m_bMark = true;
    else if (event.keyUp())
        m_bMark = false;
}

/** \brief Callback function to deal with mouse presses in visualizer
 */
void InteractiveRegisterer::mouseCallback(const pcl::visualization::MouseEvent& event, void* ptr)
{
    if (event.getType() == pcl::visualization::MouseEvent::MouseMove)
    {
        int j = event.getX() / m_WndWidth;
        int i = event.getY() / m_WndHeight;
        
        m_Viewport = i * m_Hp + j;
    }
}

/** \brief Callback function to deal with point picking actions in visualizer (shift+mouse press)
 */
void InteractiveRegisterer::ppCallback(const pcl::visualization::PointPickingEvent& event, void* ptr)
{
    if (event.getPointIndex () < 0 || !m_bMark)
        return;
    
    PointT p;
    event.getPoint(p.x, p.y, p.z);
    
    if (m_pMarkers[m_Viewport]->width < m_NumOfPoints)
    {
        m_pMarkers[m_Viewport]->push_back(p);
        
        int idx = m_pMarkers[m_Viewport]->points.size();
        string mid = "marker" + to_string(m_Viewport) + to_string(idx);
        m_pViz->addSphere(p, m_MarkerRadius, g_Colors[idx][0], g_Colors[idx][1], g_Colors[idx][2], mid, m_VIDs[m_Viewport]);
        
        // all set?
        if (m_pMarkers[m_Viewport]->width == m_NumOfPoints)
            m_Pendents--;
    }
}

/** \brief Set the position of the camera in the PCLVisualizer
 */
void InteractiveRegisterer::setDefaultCamera(VisualizerPtr pViz, int vid)
{
    vector<pcl::visualization::Camera> cameras;
    pViz->getCameras(cameras);
    
    cameras[vid].pos[2] = m_CameraDistance;
    cameras[vid].focal[2] = 1.0;
    
    cameras[vid].view[1] = -1; // y dim upside-down
    
    pViz->setCameraParameters(cameras[vid], vid);
}

