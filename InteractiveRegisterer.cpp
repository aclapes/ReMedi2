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

// Set the number of correspondences
void InteractiveRegisterer::setNumPoints(int numOfPoints)
{
	m_NumOfPoints = numOfPoints;
}

// Set the frames used to establish the correspondences
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

// Set the distribution of the viewports in the visualizer
//    wndHeight : window height (visor)
//    wndWidth : window width (visor)
//    vp : number of vertical ports (visor)
//    hp : number of horizontal ports (visor)
//    camDist : camera dist to (0,0,0) (visor)
//    markerRadius : marker sphere radius (visor)
void InteractiveRegisterer::setVisualizerParameters(int wndHeight, int wndWidth, int vp, int hp, float camDist, float markerRadius)
{
    m_WndHeight = wndHeight;
    m_WndWidth = wndWidth;
    m_Vp = vp;
    m_Hp = hp;
    m_CameraDistance = camDist;
    m_MarkerRadius = markerRadius;
}

// Manually interact with a visualizer to place the markers that
// serve as correspondences
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

// Estimate the orthogonal transformations in the n-1 views respect to the 0-th view
void InteractiveRegisterer::computeTransformations()
{
    for (int v = 0; v < m_pFrames.size(); v++)
        getTransformation(m_pMarkers[v], m_pMarkers[0], m_Transformations[v]); // v (src) to 0 (tgt)
}

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
    

void InteractiveRegisterer::registrate(vector<PointCloudPtr> pUnregClouds, vector<PointCloudPtr>& pRegClouds)
{
    pRegClouds.resize(pUnregClouds.size());
    
    for (int v = 0; v < pUnregClouds.size(); v++)
    {
        pRegClouds[v] = PointCloudPtr(new PointCloud);
        if (v == 0)
        {
            translate(pUnregClouds[v], m_pMarkers[v]->points[0], *(pRegClouds[v]));
        }
        else
        {
            PointCloudPtr pCtrCloud (new PointCloud);
            translate(pUnregClouds[v], m_pMarkers[v]->points[0], *pCtrCloud);
            pcl::transformPointCloud(*pCtrCloud, *(pRegClouds[v]), m_Transformations[v]);
        }
    }
}

//void InteractiveRegisterer::computeTransformation()
//{
//    find_transformation(lefties_, righties_, m_Transformation);
//    m_InverseTransformation = m_Transformation.inverse();
//    
//    registration(cloud_left_, cloud_right_,
//                 *aligned_cloud_left_, *aligned_cloud_right_);
//}
//
//void InteractiveRegisterer::computeFineTransformation()
//{
//    PointCloud downsampled_aligned_cloud_left, downsampled_aligned_cloud_right;
//    // TODO: implementation
//}
//
//
//
//pair<PointCloudPtr,PointCloudPtr> InteractiveRegisterer::getRegisteredClouds()
//{
//    return pair<PointCloudPtr,PointCloudPtr>(aligned_cloud_left_, aligned_cloud_right_);
//}
//

//void InteractiveRegisterer::keyboardCallback(const pcl::visualization::KeyboardEvent& keyboard_event, void*)
//{
//    cout << "the key \'" << keyboard_event.getKeyCode() << "\' (" << keyboard_event.getKeyCode() << ") was";
//    if (keyboard_event.getKeyCode ())
//        cout << "the key \'" << keyboard_event.getKeyCode() << "\' (" << keyboard_event.getKeyCode() << ") was";
//    else
//        cout << "the special key \'" << keyboard_event.getKeySym() << "\' was";
//    if (keyboard_event.keyDown())
//        cout << " pressed" << endl;
//    else
//        cout << " released" << endl;
//}
//
//
//void InteractiveRegisterer::mouseCallback(const pcl::visualization::MouseEvent& mouse_event, void*)
//{
//    m_MouseX = mouse_event.getX();
//    m_MouseY = mouse_event.getY();
//    
////    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
////    {
////    }
//    
//    if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::RightButton)
//    {
//        int x, y, z;
//        int view = (m_MouseX < 640) ? 0 : 1;
//        stringstream ss_sphere, ss_line;
//
//        if (view == 0 && lefties_->points.size() > 0)
//        {
//            x = m_MouseX;
//            y = m_MouseY;
//            z = cloud_left_->at(x,y).z;
//            
//            float distance;
//            int idx = -1;
//            for (int i = 0; i < lefties_->points.size() && idx < 0; i++)
//            {
//                distance = sqrtf(pow(x - lefties_->points[i].x, 2)
//                                 + pow(y - lefties_->points[i].y, 2)
//                                 + pow(z - lefties_->points[i].z, 2));
//                if (distance < 0.1) idx = i;
//            }
//            ss_sphere << "left sphere " << idx;
//            m_pViz->removeShape(ss_sphere.str());
//            
//            lefties_->points.erase(lefties_->points.begin() + idx);
//            lefties_->width--; // add a point in a row
//            
//            lefties_idx_.erase(lefties_idx_.begin() + idx);
//            
//            cout << "removed left" << endl;
//            
//        }
//        else if (view == 1 && righties_->points.size() > 0)
//        {
//            x = m_MouseX - 640;
//            y = m_MouseY;
//            z = cloud_left_->at(x,y).z;
//            
//            float distance;
//            int idx = -1;
//            for (int i = 0; i < righties_->points.size() && idx < 0; i++)
//            {
//                distance = sqrtf(pow(x - righties_->points[i].x, 2)
//                                 + pow(y - righties_->points[i].y, 2)
//                                 + pow(z - righties_->points[i].z, 2));
//                if (distance < 0.1) idx = i;
//            }
//            ss_sphere << "right sphere " << idx;
//            m_pViz->removeShape(ss_sphere.str());
//            
//            righties_->points.erase(righties_->points.begin() + idx);
//            righties_->width--; // add a point in a row
//            
//            righties_idx_.erase(righties_idx_.begin() + idx);
//            
//            cout << "removed right" << endl;
//        }
//    }
//}
//
//
//void InteractiveRegisterer::ppCallback(const pcl::visualization::PointPickingEvent& pointpicking_event, void*)
//{       
//    // Error handling
//    if (pointpicking_event.getPointIndex () == -1) return;
//        
//    float x, y, z;
//    pointpicking_event.getPoint(x,y,z);
//        
//    cout << "left + shift button pressed @ " << x << " , " << y << " , " << z << endl;
//    
//    int view = (m_MouseX < 640) ? 0 : 1;
//
//    // If not all points picked up in both sides, continue
//    stringstream ss_sphere, ss_line;
//
//    if (view == 0)
//    {
//        float distance;
//        int idx = -1;
//        for (int i = 0; i < lefties_->points.size() && idx < 0; i++)
//        {
//            distance = sqrtf(pow(x - lefties_->points[i].x, 2)
//                             + pow(y - lefties_->points[i].y, 2)
//                             + pow(z - lefties_->points[i].z, 2));
//            if (distance < 0.1) idx = i;
//        }
//        
//        if (idx >=0)
//        {
//            ss_sphere << "left sphere " << idx;
//            m_pViz->removeShape(ss_sphere.str());
//            
//            lefties_->points.erase(lefties_->points.begin() + idx);
//            lefties_->width--; // add a point in a row
//            
//            lefties_idx_.erase(lefties_idx_.begin() + idx);
//            
//            cout << "removed left" << endl;
//        }
//        else if (lefties_->points.size() < num_points_)
//        {
//            ss_sphere << "left sphere " << lefties_->points.size();
//        
//            if (lefties_->empty()) m_tLeft << x, y, z, 1;
//            
//            int idx = lefties_->points.size();
//            m_pViz->addSphere(PointT(x,y,z), 0.03, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_left_);
//                
//            lefties_->points.push_back(PointT(x,y,z));
//            lefties_->width++; // add a point in a row
//                
//            lefties_idx_.push_back(pointpicking_event.getPointIndex());
//            
//            cout << "added left" << endl;
//        }
//    }
//    else if (view == 1)
//    {
//        float distance;
//        int idx = -1;
//        for (int i = 0; i < righties_->points.size() && idx < 0; i++)
//        {
//            distance = sqrtf(pow(x - righties_->points[i].x, 2)
//                             + pow(y - righties_->points[i].y, 2)
//                             + pow(z - righties_->points[i].z, 2));
//            if (distance < 0.1) idx = i;
//        }
//        
//        if (idx >= 0)
//        {
//            ss_sphere << "right sphere " << idx;
//            m_pViz->removeShape(ss_sphere.str());
//            
//            righties_->points.erase(righties_->points.begin() + idx);
//            righties_->width--; // add a point in a row
//            
//            righties_idx_.erase(righties_idx_.begin() + idx);
//            
//            cout << "removed right" << endl;
//        }
//        else if (righties_->points.size() < num_points_)
//        {
//            ss_sphere << "right sphere " << righties_->points.size();
//            
//            if (righties_->empty()) m_tRight << x, y, z, 1;                
//            
//            int idx = righties_->points.size();
//            m_pViz->addSphere(PointT(x,y,z), 0.03, colors[idx][0], colors[idx][1], colors[idx][2], ss_sphere.str(), viewport_right_);
//                
//            righties_->points.push_back(PointT(x,y,z));
//            righties_->width++;
//                
//            righties_idx_.push_back(pointpicking_event.getPointIndex());
//            
//            cout << "added right" << endl;
//        }
//    }
//    
//    if (!must_translate_ && lefties_->points.size() == num_points_ && righties_->points.size() == num_points_)
//    {
//        //pcl::PCDWriter writer;
//        //writer.write("lefties.pcd", *lefties_);
//        //writer.write("righties.pcd", *righties_);
//        //    
//        //cv::FileStorage fs;
//        //fs.open("indices.yml", cv::FileStorage::WRITE);
//        //fs << "lefties_idx_" << lefties_idx_;
//        //fs << "righties_idx_" << righties_idx_;
//            
//        must_translate_ = true;
//        //cout << "All " << num_points_ << " have been defined." << endl;
//    }
//}
//
//
//

//////////////////////////////////////////////////////////////////////////////////
/** \brief Find the transformation needed to align two sets of 3-D points (minimum 3)
* \param src the source points
* \param tgt the target points
* \param transformation the resultant transform between source and target
*/
void InteractiveRegisterer::getTransformation(const PointCloudPtr pSrcMarkersCloud, const PointCloudPtr pTgtMarkersCloud, Eigen::Matrix4f& T)
{
    Eigen::Vector4f centroidSrc, centroidTgt;
    pcl::compute3DCentroid(*pSrcMarkersCloud, centroidSrc);
    pcl::compute3DCentroid(*pTgtMarkersCloud, centroidTgt);
        
    PointCloudPtr pSrcCenteredMarkersCloud (new PointCloud);
    PointCloudPtr pTgtCenteredMarkersCloud (new PointCloud);
        
    translate(pSrcMarkersCloud, centroidSrc, *pSrcCenteredMarkersCloud);
    translate(pTgtMarkersCloud, centroidTgt, *pTgtCenteredMarkersCloud);
        
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

    T <<
        r.at<float>(0,0), r.at<float>(0,1), r.at<float>(0,2),  0,
        r.at<float>(1,0), r.at<float>(1,1), r.at<float>(1,2),  0,
        r.at<float>(2,0), r.at<float>(2,1), r.at<float>(2,2),  0,
                       0,                0,                0,  1;
        
    cout << T << endl;
}

////
////void InteractiveRegisterer::align(
////    const PointCloudPtr cloud_src, const PointCloudPtr cloud_tgt,
////	PointCloudPtr ref_points_src, PointCloudPtr ref_points_tgt, 
////	PointCloudPtr cloud_src_aligned, PointCloudPtr cloud_tgt_aligned)
////{
////    Eigen::Vector4f centroid_cloud_src, centroid_cloud_tgt;
////    pcl::compute3DCentroid(*ref_points_src, centroid_cloud_src);
////    pcl::compute3DCentroid(*ref_points_tgt, centroid_cloud_tgt);
////        
////    PointCloudPtr pCloudSrcCentered (new PointCloud);
////    translate(cloud_src, centroid_cloud_src, *pCloudSrcCentered);
////    translate(cloud_tgt, centroid_cloud_tgt, *cloud_tgt_aligned);
////        
////    pcl::transformPointCloud(*pCloudSrcCentered, *cloud_src_aligned, m_Transformation);
////}
//
////void InteractiveRegisterer::align(DepthFrame dFrameA, DepthFrame dFrameB) // public
////{        
////	PointCloudPtr cloud_left_	(new PointCloud);
////	PointCloudPtr cloud_right_ (new PointCloud);
////
////	dFrameA.getPointCloud(*cloud_left_);
////	dFrameB.getPointCloud(*cloud_right_);
//////
//////    m_pViz->addPointCloud (cloud_left_, "Left cloud", viewport_left_);
//////    m_pViz->addPointCloud (cloud_right_, "Right cloud", viewport_right_);
//////        
//////    while (!must_translate_)
//////    {
//////        m_pViz->spinOnce(100);
//////    }
//////    stop(*m_pViz);
//////
////    PointCloudPtr cloud_src_aligned (new PointCloud), cloud_tgt_aligned (new PointCloud);
////    align(cloud_left_, cloud_right_, lefties_, righties_, cloud_src_aligned, cloud_tgt_aligned);
////        
////    // Visualize and that shit
////	visualizeRegistration(cloud_src_aligned, cloud_tgt_aligned);
////}
//
//
//void InteractiveRegisterer::saveTransformation(string filePath)
//{
//	cv::Mat tLeftMat, tRightMat, transfMat;
//	cv::eigen2cv(m_tLeft, tLeftMat);
//	cv::eigen2cv(m_tRight, tRightMat);
//	cv::eigen2cv(m_Transformation, transfMat);
//
//	cv::FileStorage fs (filePath, cv::FileStorage::WRITE);
//	fs << "tLeftMat" << tLeftMat;
//	fs << "tRightMat" << tRightMat;
//	fs << "transfMat" << transfMat;
//
//	fs.release();
//}
//
//
//bool InteractiveRegisterer::loadTransformation(string filePath)
//{
//	cv::Mat tLeftMat, tRightMat, transfMat;
//	cv::FileStorage fs (filePath, cv::FileStorage::READ);
//    
//    if (!fs.isOpened())
//        return false;
//    
//	fs["tLeftMat"] >> tLeftMat;
//	fs["tRightMat"] >> tRightMat;
//	fs["transfMat"] >> transfMat;
//
//	fs.release();
//
//	if (transfMat.rows > 0 && transfMat.cols > 0)
//	{
//		cv::cv2eigen(transfMat, m_Transformation);
//        m_InverseTransformation = m_Transformation.inverse();
//		cv::cv2eigen(tLeftMat, m_tLeft);
//		cv::cv2eigen(tRightMat, m_tRight);
//        
//		return true;
//	}
//	else
//	{
//		return false;
//	}
//}
//
//PointT InteractiveRegisterer::registration(PointT point, int viewpoint)
//{
//    PointT regPoint;
//    
//    deregistration(point, regPoint, viewpoint);
//    
//    return regPoint;
//}
//
//void InteractiveRegisterer::registration(PointT point, PointT& regPoint, int viewpoint)
//{
//    PointCloudPtr pCloud (new PointCloud);
//    PointCloudPtr pRegCloud (new PointCloud);
//    
//    pCloud->push_back(point);
//    pCloud->height = pCloud->width = 1;
//    
//    registration(pCloud, *pRegCloud, viewpoint);
//    
//    regPoint = pRegCloud->points[0];
//}
//
//void InteractiveRegisterer::registration(PointT pointA, PointT pointB, PointT& regPointA, PointT& regPointB)
//{
//    PointCloudPtr pCloudA (new PointCloud);
//    PointCloudPtr pCloudB (new PointCloud);
//    PointCloudPtr pRegCloudA (new PointCloud);
//    PointCloudPtr pRegCloudB (new PointCloud);
//    
//    pCloudA->push_back(pointA);
//    pCloudA->height = pCloudA->width = 1;
//    pCloudB->push_back(pointB);
//    pCloudB->height = pCloudB->width = 1;
//
//    registration(pCloudA, pCloudB, *pRegCloudA, *pRegCloudB);
//    
//    regPointA = pRegCloudA->points[0];
//    regPointB = pRegCloudB->points[0];
//}
//
//void InteractiveRegisterer::registration(PointCloudPtr pCloudA, PointCloudPtr pCloudB, PointCloud& regCloudA, PointCloud& regCloudB)
//{
//    registration(pCloudA, regCloudA, 0);
//    registration(pCloudB, regCloudB, 1);
//}
//
//void InteractiveRegisterer::registration(PointCloudPtr pCloud, PointCloud& regCloud, int view)
//{
//    if (view == 0)
//    {
//        PointCloudPtr pCtrCloud (new PointCloud);
//        translate(pCloud, m_tLeft, *pCtrCloud);
//        pcl::transformPointCloud(*pCtrCloud, regCloud, m_Transformation);
//    }
//    else
//    {
//        translate(pCloud, m_tRight, regCloud);
//    }
//}
//
//void InteractiveRegisterer::registration(vector<PointCloudPtr> pCloudsA, vector<PointCloudPtr> pCloudsB, vector<PointCloudPtr>& pRegCloudsA, vector<PointCloudPtr>& pRegCloudsB)
//{
//    for (int i = 0; i < pRegCloudsA.size(); i++)
//    {
//        PointCloudPtr pRegCloudA (new PointCloud);
//        registration(pCloudsA[i], *pRegCloudA, 0);
//        
//        pCloudsA.push_back(pRegCloudA);
//    }
//    
//    for (int i = 0; i < pRegCloudsB.size(); i++)
//    {
//        PointCloudPtr pRegCloudB (new PointCloud);
//        registration(pCloudsB[i], *pRegCloudB, 0);
//        
//        pCloudsA.push_back(pRegCloudB);
//    }
//}
//
//PointT InteractiveRegisterer::deregistration(PointT regPoint, int viewpoint)
//{
//    PointT point;
//    
//    deregistration(regPoint, point, viewpoint);
//    
//    return point;
//}
//
//void InteractiveRegisterer::deregistration(PointT regPoint, PointT& point, int viewpoint)
//{
//    PointCloudPtr pRegCloud (new PointCloud);
//    PointCloudPtr pCloud (new PointCloud);
//    
//    pRegCloud->push_back(regPoint);
//    pRegCloud->height = pRegCloud->width = 1;
//    
//    deregistration(pRegCloud, *pCloud, viewpoint);
//    
//    point = pCloud->points[0];
//}
//
//void InteractiveRegisterer::deregistration(PointT regPointA, PointT regPointB, PointT& pointA, PointT& pointB)
//{
//    PointCloudPtr pRegCloudA (new PointCloud);
//    PointCloudPtr pRegCloudB (new PointCloud);
//    PointCloudPtr pCloudA (new PointCloud);
//    PointCloudPtr pCloudB (new PointCloud);
//    
//    pRegCloudA->push_back(regPointA);
//    pRegCloudA->height = pRegCloudA->width = 1;
//    pRegCloudB->push_back(regPointB);
//    pRegCloudB->height = pRegCloudB->width = 1;
//
//    deregistration(pRegCloudA, pRegCloudB, *pCloudA, *pCloudB);
//    
//    pointA = pCloudA->points[0];
//    pointB = pCloudB->points[0];
//}
//
//void InteractiveRegisterer::deregistration(PointCloudPtr pRegCloudA, PointCloudPtr pRegCloudB, PointCloud& cloudA, PointCloud& cloudB)
//{
//    if (!pRegCloudA->empty()) deregistration(pRegCloudA, cloudA, 0);
//    if (!pRegCloudB->empty()) deregistration(pRegCloudB, cloudB, 1);
//}
//
//void InteractiveRegisterer::deregistration(PointCloudPtr pRegCloud, PointCloud& cloud, int viewpoint)
//{
//    if (viewpoint == 0)
//    {
//        PointCloudPtr pCtrCloud (new PointCloud);
//        pcl::transformPointCloud(*pRegCloud, *pCtrCloud, m_InverseTransformation);
//        translate(pCtrCloud, -m_tLeft,  cloud);
//    }
//    else
//    {
//        translate(pRegCloud, -m_tRight, cloud);
//    }
//}
//
//void InteractiveRegisterer::deregistration(vector<PointCloudPtr> pRegCloudsA, vector<PointCloudPtr> pRegCloudsB, vector<PointCloudPtr>& pCloudsA, vector<PointCloudPtr>& pCloudsB)
//{
//    for (int i = 0; i < pRegCloudsA.size(); i++)
//    {
//        PointCloudPtr pCloudA (new PointCloud);
//        deregistration(pRegCloudsA[i], *pCloudA, 0);
//        
//        pCloudsA.push_back(pCloudA);
//    }
//    
//    for (int i = 0; i < pRegCloudsB.size(); i++)
//    {
//        PointCloudPtr pCloudB (new PointCloud);
//        deregistration(pRegCloudsB[i], *pCloudB, 1);
//        
//        pCloudsB.push_back(pCloudB);
//    }
//}
//
//void InteractiveRegisterer::registration(DepthFrame::Ptr frameA, DepthFrame::Ptr frameB,
//	PointCloud& regCloudA, PointCloud& regCloudB, 
//	bool bBackgroundPoints, bool bUserPoints)
//{    
//	PointCloudPtr pCloudA (new PointCloud);
//	PointCloudPtr pCloudB (new PointCloud);
//
//	if (bBackgroundPoints && bUserPoints)
//	{
//		frameA->getPointCloud(*pCloudA);
//		frameB->getPointCloud(*pCloudB);
//	}
//	else if (bBackgroundPoints && !bUserPoints)
//	{
//		frameA->getUserFreePointCloud(*pCloudA);
//		frameB->getUserFreePointCloud(*pCloudB);
//	}
//	else if (!bBackgroundPoints && bUserPoints)
//	{
//		frameA->getForegroundPointCloud(*pCloudA);
//		frameB->getForegroundPointCloud(*pCloudB);
//	}
//	else
//	{
//		frameA->getForegroundUserFreePointCloud(*pCloudA);
//		frameB->getForegroundUserFreePointCloud(*pCloudB);
//	}
//    
////    pcl::visualization::PCLVisualizer::Ptr viz (new pcl::visualization::PCLVisualizer);
////    viz->addPointCloud(pCloudA, "A");
////    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "A");
//
////    PointCloudPtr pCtrCloudA (new PointCloud);
////    translate(pCloudA, m_tLeft, *pCtrCloudA);
////    translate(pCloudB, m_tRight, regCloudB);
////        
////    pcl::transformPointCloud(*pCtrCloudA, regCloudA, m_Transformation);
//    
//    registration(pCloudA, pCloudB, regCloudA, regCloudB);
////    
////    viz->addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(&regCloudA), "rA");
////    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "rA");
////    viz->spin();
//}
//
//void InteractiveRegisterer::registration(ColorDepthFrame::Ptr frameA, ColorDepthFrame::Ptr frameB, PointCloud& regCloudA, PointCloud& regCloudB, bool bBackgroundPoints, bool bUserPoints)
//{
//	PointCloudPtr pCloudA (new PointCloud);
//	PointCloudPtr pCloudB (new PointCloud);
//    
//	if (bBackgroundPoints && bUserPoints)
//	{
//		frameA->getPointCloud(*pCloudA);
//		frameB->getPointCloud(*pCloudB);
//	}
//	else if (bBackgroundPoints && !bUserPoints)
//	{
//		frameA->getUserFreePointCloud(*pCloudA);
//		frameB->getUserFreePointCloud(*pCloudB);
//	}
//	else if (!bBackgroundPoints && bUserPoints)
//	{
//		frameA->getForegroundPointCloud(*pCloudA);
//		frameB->getForegroundPointCloud(*pCloudB);
//	}
//	else
//	{
//		frameA->getForegroundUserFreePointCloud(*pCloudA);
//		frameB->getForegroundUserFreePointCloud(*pCloudB);
//	}
//    
//    //    pcl::visualization::PCLVisualizer::Ptr viz (new pcl::visualization::PCLVisualizer);
//    //    viz->addPointCloud(pCloudA, "A");
//    //    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "A");
//    
//    //    PointCloudPtr pCtrCloudA (new PointCloud);
//    //    translate(pCloudA, m_tLeft, *pCtrCloudA);
//    //    translate(pCloudB, m_tRight, regCloudB);
//    //
//    //    pcl::transformPointCloud(*pCtrCloudA, regCloudA, m_Transformation);
//    
//    registration(pCloudA, pCloudB, regCloudA, regCloudB);
//    //
//    //    viz->addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(&regCloudA), "rA");
//    //    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "rA");
//    //    viz->spin();
//}
//
//
//void InteractiveRegisterer::visualizeRegistration(PointCloudPtr cloudA, PointCloudPtr cloudB)
//{
//    pcl::visualization::PCLVisualizer::Ptr pViz (new pcl::visualization::PCLVisualizer);
//	visualizeRegistration(pViz, cloudA, cloudB);
//}
//
//void InteractiveRegisterer::visualizeRegistration(pcl::visualization::PCLVisualizer::Ptr pViz, PointCloudPtr cloudA, PointCloudPtr cloudB)
//{
//	pViz->removePointCloud("cloud left");
//    pViz->removePointCloud("cloud right");
//
//    pViz->addPointCloud (cloudA, "cloud left");
//    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0, 0, "cloud left");
//    pViz->addPointCloud (cloudB, "cloud right");
//    pViz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, "cloud right");
//    
//    pViz->spin();
//}
//
//void InteractiveRegisterer::visualizeRegistration(DepthFrame::Ptr pDepthFrameA, DepthFrame::Ptr pDepthFrameB)
//{
//	PointCloudPtr pCloudA (new PointCloud);
//	PointCloudPtr pCloudB (new PointCloud);
//    
//	registration(pDepthFrameA, pDepthFrameB, *pCloudA, *pCloudB);
//    
//    visualizeRegistration(pCloudA, pCloudB);
//}
//
//PointT InteractiveRegisterer::getLeftRefPoint()
//{
//    return EigenToPointXYZ(m_tLeft);
//}
//
//PointT InteractiveRegisterer::getRightRefPoint()
//{
//    return EigenToPointXYZ(m_tRight);
//}

// Callback function to deal with keyboard presses in visualizer
void InteractiveRegisterer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* ptr)
{
    if (event.keyDown() && event.getKeySym() == "s")
        m_bMark = true;
    else if (event.keyUp())
        m_bMark = false;
}

// Callback function to deal with mouse presses in visualizer
void InteractiveRegisterer::mouseCallback(const pcl::visualization::MouseEvent& event, void* ptr)
{
    if (event.getType() == pcl::visualization::MouseEvent::MouseMove)
    {
        int j = event.getX() / m_WndWidth;
        int i = event.getY() / m_WndHeight;
        
        m_Viewport = i * m_Hp + j;
    }
}

// Callback function to deal with point picking actions in visualizer (shift+mouse press)
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

void InteractiveRegisterer::setDefaultCamera(VisualizerPtr pViz, int vid)
{
    vector<pcl::visualization::Camera> cameras;
    pViz->getCameras(cameras);
    
    cameras[vid].pos[2] = m_CameraDistance;
    cameras[vid].focal[2] = 1.0;
    
    cameras[vid].view[1] = -1; // y dim upside-down
    
    pViz->setCameraParameters(cameras[vid], vid);
}

