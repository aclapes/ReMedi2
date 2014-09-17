//
//  ObjectRecognizer.cpp
//  remedi2
//
//  Created by Albert Clapés on 16/09/14.
//
//

#include "ObjectRecognizer.h"

//
// <ColorPointT, FPFHSignatureT>
//

ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::ObjectRecognizer()
{
    
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::ObjectRecognizer(const ObjectRecognizer& rhs)
{
    *this = rhs;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs)
{
    *this = rhs;
}


ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs)
{
    if (this != &rhs)
    {
        m_ObjectModels = rhs.m_ObjectModels;
        m_CloudjectModels = rhs.m_CloudjectModels;
        m_RecognitionStrategy = rhs.m_RecognitionStrategy;
        m_CloudjectDetections = rhs.m_CloudjectDetections;
        
        m_LeafSize = rhs.m_LeafSize;
        m_LeafSizeModel = rhs.m_LeafSizeModel;
        m_NormalRadius = rhs.m_NormalRadius;
        m_NormalRadiusModel = rhs.m_NormalRadiusModel;
        m_PfhRadius = rhs.m_PfhRadius;
        m_PfhRadiusModel = rhs.m_PfhRadiusModel;
    }
    
    return *this;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs)
{
    m_ObjectModels = rhs.getInputObjectModels();
    m_RecognitionStrategy = rhs.getRecognitionStrategy();
    
    m_LeafSize = rhs.getCloudjectsLeafSize();
    m_LeafSizeModel = rhs.getCloudjectModelsLeafSize();
    m_NormalRadius = rhs.getCloudjectsNormalRadius();
    m_NormalRadiusModel = rhs.getCloudjectModelsNormalRadius();
    m_PfhRadius = rhs.getCloudjectsPfhRadius();
    m_PfhRadiusModel = rhs.getCloudjectModelsPfhRadius();
    
    return *this;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setInputObjectModels(vector<ObjectModel<pcl::PointXYZRGB>::Ptr> models)
{
    m_ObjectModels = models;
    
    m_CloudjectModels.resize(models.size());
    for (int m = 0; m < models.size(); m++)
    {
        int oid = models[m]->getID();
        string name = models[m]->getName();
        vector<ColorPointCloud::Ptr> views = models[m]->getViews();
        
        LFCloudjectModel<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr pCloudjectModel (new LFCloudjectModel<pcl::PointXYZRGB, pcl::FPFHSignature33>(oid, name, views));
        m_CloudjectModels[m] = pCloudjectModel;
    }
}

vector<ObjectModel<pcl::PointXYZRGB>::Ptr> ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getInputObjectModels() const
{
    return m_ObjectModels;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectsLeafSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectModelsLeafSize(float leafSize)
{
    m_LeafSizeModel = leafSize;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectsNormalRadius(float r)
{
    m_NormalRadius = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectModelsNormalRadius(float r)
{
    m_NormalRadiusModel = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectsPfhRadius(float r)
{
    m_PfhRadius = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setCloudjectModelsPfhRadius(float r)
{
    m_PfhRadiusModel = r;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectsLeafSize() const
{
    return m_LeafSize;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectModelsLeafSize() const
{
    return m_LeafSizeModel;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectsNormalRadius() const
{
    return m_NormalRadius;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectModelsNormalRadius() const
{
    return m_NormalRadiusModel;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectsPfhRadius() const
{
    return m_PfhRadius;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getCloudjectModelsPfhRadius() const
{
    return m_PfhRadiusModel;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::create()
{
    for (int m = 0; m < m_ObjectModels.size(); m++)
        m_CloudjectModels[m]->describe(m_NormalRadiusModel, m_PfhRadiusModel, m_LeafSizeModel);
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections)
{
    m_CloudjectDetections.resize(detections.size());
    for (int i = 0; i < detections.size(); i++)
    {
        LFCloudject<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr pCloudject ( new LFCloudject<pcl::PointXYZRGB, pcl::FPFHSignature33>(detections[i]) );
        m_CloudjectDetections[i] = pCloudject;
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setRecognitionStrategy(int strategy)
{
    m_RecognitionStrategy = strategy;
}

int ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getRecognitionStrategy() const
{
    return m_RecognitionStrategy;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
//    // Segment objects
//    vector<vector<ColorPointCloudPtr> > detections;
//    detect(detections);
//    
//    // Find inter-view correspondences
//    vector<vector<pair<int,ColorPointCloudPtr> > > correspondences;
//    if (m_RecognitionStrategy == RECOGNITION_MULTIOCULAR_AVERAGE)
//        findCorrespondences(detections, m_CorrespenceDist, correspondences);
//    else
//        findCorrespondences(detections, 0, correspondences);
//    
//
//    
    // Recognize the initialized cloudjects
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius, m_LeafSize);
        
        int maxScoreIdx;
        float maxScore = 0;
        
        for (int m = 0; m < m_CloudjectModels.size(); m++)
        {
            float score = m_CloudjectModels[m]->getScore(m_CloudjectDetections[i]);
            if (score >= maxScore)
            {
                maxScoreIdx = m;
                maxScore    = score;
            }
        }
        
        m_CloudjectDetections[i]->setID(m_CloudjectModels[maxScoreIdx]->getID());
        m_CloudjectDetections[i]->setName(m_CloudjectModels[maxScoreIdx]->getName());
    }
    
    // Cloudjects to output format
    
    recognitions.resize(NUM_OF_VIEWS);
    for (int v = 0; v < recognitions.size(); v++)
        recognitions[v].resize(OD_NUM_OF_OBJECTS + 1);
    
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        int id = m_CloudjectDetections[i]->getID();
        vector<int> viewIDs = m_CloudjectDetections[i]->getViewIDs();
        vector<pcl::PointXYZ> positions = m_CloudjectDetections[i]->getPositions();
        
        for (int v = 0; v < viewIDs.size(); v++)
            recognitions[v][id].push_back(positions[v]);
    }
}


//
// <ColorPointT, PFHRGBSignatureT>
//

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::ObjectRecognizer()
{
    
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs)
{
    *this = rhs;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs)
{
    *this = rhs;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs)
{
    if (this != &rhs)
    {
        m_ObjectModels = rhs.m_ObjectModels;
        m_CloudjectModels = rhs.m_CloudjectModels;
        m_RecognitionStrategy = rhs.m_RecognitionStrategy;
        m_CloudjectDetections = rhs.m_CloudjectDetections;
    }
    
    return *this;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs)
{
    m_ObjectModels = rhs.getInputObjectModels();
    m_RecognitionStrategy = rhs.getRecognitionStrategy();
    
    m_LeafSize = rhs.getCloudjectsLeafSize();
    m_LeafSizeModel = rhs.getCloudjectModelsLeafSize();
    m_NormalRadius = rhs.getCloudjectsNormalRadius();
    m_NormalRadiusModel = rhs.getCloudjectModelsNormalRadius();
    m_PfhRadius = rhs.getCloudjectsPfhRadius();
    m_PfhRadiusModel = rhs.getCloudjectModelsPfhRadius();
    
    return *this;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setInputObjectModels(vector<ObjectModel<pcl::PointXYZRGB>::Ptr> models)
{
    m_ObjectModels = models;
}

vector<ObjectModel<pcl::PointXYZRGB>::Ptr> ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getInputObjectModels() const
{
    return m_ObjectModels;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectsLeafSize(float leafSize)
{
    m_LeafSize = leafSize;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectModelsLeafSize(float leafSize)
{
    m_LeafSizeModel = leafSize;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectsNormalRadius(float r)
{
    m_NormalRadius = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectModelsNormalRadius(float r)
{
    m_NormalRadiusModel = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectsPfhRadius(float r)
{
    m_PfhRadius = r;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setCloudjectModelsPfhRadius(float r)
{
    m_PfhRadiusModel = r;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectsLeafSize() const
{
    return m_LeafSize;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectModelsLeafSize() const
{
    return m_LeafSizeModel;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectsNormalRadius() const
{
    return m_NormalRadius;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectModelsNormalRadius() const
{
    return m_NormalRadiusModel;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectsPfhRadius() const
{
    return m_PfhRadius;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getCloudjectModelsPfhRadius() const
{
    return m_PfhRadiusModel;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::create()
{
    m_CloudjectModels.clear();
    for (int m = 0; m < m_ObjectModels.size(); m++)
    {
        int oid = m_ObjectModels[m]->getID();
        string name = m_ObjectModels[m]->getName();
        vector<ColorPointCloud::Ptr> views = m_ObjectModels[m]->getViews();
        
        LFCloudjectModel<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr pCloudjectModel (new LFCloudjectModel<pcl::PointXYZRGB, pcl::PFHRGBSignature250>(oid, name, views));
        m_CloudjectModels.push_back(pCloudjectModel);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections)
{
    m_CloudjectDetections.clear();
    for (int i = 0; i < detections.size(); i++)
    {
        LFCloudject<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr pCloudject ( new LFCloudject<pcl::PointXYZRGB, pcl::PFHRGBSignature250>(detections[i]) );
        m_CloudjectDetections.push_back(pCloudject);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setRecognitionStrategy(int strategy)
{
    m_RecognitionStrategy = strategy;
}

int ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getRecognitionStrategy() const
{
    return m_RecognitionStrategy;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        int maxScoreIdx;
        float maxScore = 0;
        
        for (int m = 0; m < m_CloudjectModels.size(); m++)
        {
            float score = m_CloudjectModels[m]->getScore(m_CloudjectDetections[i]);
            if (score >= maxScore)
            {
                maxScoreIdx = m;
                maxScore    = score;
            }
        }
        
        m_CloudjectDetections[i]->setID(m_CloudjectModels[maxScoreIdx]->getID());
        m_CloudjectDetections[i]->setName(m_CloudjectModels[maxScoreIdx]->getName());
    }
    
    // Cloudjects to output format
    
    recognitions.resize(NUM_OF_VIEWS);
    for (int v = 0; v < recognitions.size(); v++)
        recognitions[v].resize(OD_NUM_OF_OBJECTS);
    
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        int id = m_CloudjectDetections[i]->getID();
        vector<int> viewIDs = m_CloudjectDetections[i]->getViewIDs();
        vector<pcl::PointXYZ> positions = m_CloudjectDetections[i]->getPositions();
        
        for (int v = 0; v < viewIDs.size(); v++)
            recognitions[v][id].push_back(positions[v]);
    }
}