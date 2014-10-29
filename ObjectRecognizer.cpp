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
: m_LeafSize(0.005), m_LeafSizeModel(0.005), m_NormalRadius(0.05), m_NormalRadiusModel(0.05), m_PfhRadius(0.15), m_PfhRadiusModel(0.15)
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
        m_ObjectRejections = rhs.m_ObjectRejections;
        
        m_LeafSize = rhs.m_LeafSize;
        m_LeafSizeModel = rhs.m_LeafSizeModel;
        m_NormalRadius = rhs.m_NormalRadius;
        m_NormalRadiusModel = rhs.m_NormalRadiusModel;
        m_PfhRadius = rhs.m_PfhRadius;
        m_PfhRadiusModel = rhs.m_PfhRadiusModel;
        
        m_PointScoreRejectionThreshold = rhs.m_PointScoreRejectionThreshold;

        m_RecognitionStrategy = rhs.m_RecognitionStrategy;
        m_RecognitionConsensus = rhs.m_RecognitionConsensus;
        
        m_CloudjectModels = rhs.m_CloudjectModels;
        
        m_CloudjectDetections = rhs.m_CloudjectDetections;
    }
    
    return *this;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs)
{
    m_ObjectModels = rhs.getInputObjectModels();
    setInputObjectModels(m_ObjectModels);
    m_ObjectRejections = rhs.getInputObjectRejections();
    
    m_LeafSize = rhs.getCloudjectsLeafSize();
    m_LeafSizeModel = rhs.getCloudjectModelsLeafSize();
    m_NormalRadius = rhs.getCloudjectsNormalRadius();
    m_NormalRadiusModel = rhs.getCloudjectModelsNormalRadius();
    m_PfhRadius = rhs.getCloudjectsPfhRadius();
    m_PfhRadiusModel = rhs.getCloudjectModelsPfhRadius();
    
    m_PointScoreRejectionThreshold = rhs.getPointScoreRejectionThreshold();
    
    m_RecognitionStrategy = rhs.getRecognitionStrategy();
    m_RecognitionConsensus = rhs.getRecognitionConsensus();
    
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

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setInputObjectRejections(vector<float> rejections)
{
    m_ObjectRejections = rejections;
}

vector<float> ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getInputObjectRejections() const
{
    return m_ObjectRejections;
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

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setPointScoreRejectionThreshold(float t)
{
    m_PointScoreRejectionThreshold = t;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getPointScoreRejectionThreshold() const
{
    return m_PointScoreRejectionThreshold;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::create()
{
    for (int m = 0; m < m_CloudjectModels.size(); m++)
    {
        m_CloudjectModels[m]->describe(m_NormalRadiusModel, m_PfhRadiusModel, m_LeafSizeModel);
        m_CloudjectModels[m]->setPointScoreRejectionThreshold(m_PointScoreRejectionThreshold);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections)
{
    m_CloudjectDetections.resize(detections.size());
    for (int i = 0; i < detections.size(); i++)
    {
        LFCloudject<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr pCloudject ( new LFCloudject<pcl::PointXYZRGB, pcl::FPFHSignature33>( detections[i]) );
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

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::setRecognitionConsensus(int consensus)
{
    m_RecognitionConsensus = consensus;
}

int ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getRecognitionConsensus() const
{
    return m_RecognitionConsensus;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::interviewConsensus(std::vector<pcl::PointXYZ> positions, std::vector<float> values)
{
    if (m_RecognitionStrategy == 0)
    {
        float valuesAcc = 0.f;
        
        for (int v = 0; v < values.size(); v++)
            valuesAcc += values[v];
        
        return valuesAcc / values.size();
    }
    else
    {
        float wtValuesAcc = 0.f;
        float weightsAcc = 0.f;
        
        for (int v = 0; v < values.size(); v++)
        {
            float d = sqrtf(powf(positions[v].x,2) + powf(positions[v].y,2) + powf(positions[v].z,2));
            wtValuesAcc += (1.0 / d) * values[v];
            weightsAcc += (1.0 / d);
        }
        
        return (weightsAcc > 0) ? (wtValuesAcc / weightsAcc) : 0;
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::getScores(vector<vector<int> >& vids, vector<vector<pcl::PointXYZ> >& positions, vector<vector<vector<float> > >& scores)
{
    vids.clear();
    positions.clear();
    scores.clear();
    
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        m_CloudjectDetections[i]->downsample(m_LeafSize);
        m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius);
        
        vector<int> detectionViews = m_CloudjectDetections[i]->getViewIDs();
        
        // views
        vids.push_back(detectionViews);
        
        // positions
        positions.push_back(m_CloudjectDetections[i]->getPositions());
        
        // scores
        vector<vector<float> > detectionScores (detectionViews.size(), std::vector<float>(m_CloudjectModels.size()));
        for (int m = 0; m < m_CloudjectModels.size(); m++)
        {
            std::vector<float> detectionModelScores;
            m_CloudjectModels[m]->getScores(m_CloudjectDetections[i], detectionModelScores);
            
            for (int v = 0; v < detectionModelScores.size(); v++)
                detectionScores[v][m] = detectionModelScores[v];
        }
        scores.push_back(detectionScores);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
    // Recognize the initialized cloudjects
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        m_CloudjectDetections[i]->downsample(m_LeafSize);
        m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius);
        
        int maxScoreIdx;
        float maxScore = 0;
        
        for (int m = 0; m < m_CloudjectModels.size(); m++)
        {
            std::vector<float> scores;
            m_CloudjectModels[m]->getScores(m_CloudjectDetections[i], scores);
            
            float consensusScore = interviewConsensus(m_CloudjectDetections[i]->getPositions(), scores);
            if (consensusScore >= maxScore)
            {
                maxScoreIdx = m;
                maxScore    = consensusScore;
            }
        }
        
        m_CloudjectDetections[i]->setID(m_CloudjectModels[maxScoreIdx]->getID());
        m_CloudjectDetections[i]->setName(m_CloudjectModels[maxScoreIdx]->getName());
    }
    
    // Cloudjects to output format
    
    recognitions.clear();
    
    recognitions.resize(NUM_OF_VIEWS);
    for (int v = 0; v < recognitions.size(); v++)
        recognitions[v].resize(OD_NUM_OF_OBJECTS + 1);
    
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        int id = m_CloudjectDetections[i]->getID();
        vector<int> viewIDs = m_CloudjectDetections[i]->getViewIDs();
        vector<pcl::PointXYZ> positions = m_CloudjectDetections[i]->getPositions();
        
        for (int v = 0; v < viewIDs.size(); v++)
            recognitions[viewIDs[v]][id].push_back(positions[v]);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>::recognize(std::vector<std::vector< int > > vids, std::vector<std::vector< pcl::PointXYZ > > positions, std::vector<std::vector< std::vector<float> > > scores, vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
    recognitions.clear();
    recognitions.resize(NUM_OF_VIEWS, std::vector<std::vector<pcl::PointXYZ> >(OD_NUM_OF_OBJECTS + 1));

    int ni = vids.size(); // num of instances
    for (int i = 0; i < ni; i++)
    {
        int nv = vids[i].size(); // num of views
        
        // If multiocular consensus, first consensuate the scores and overwrite the individual scores
        
        if (m_RecognitionStrategy == RECOGNITION_MULTIOCULAR)
        {
            // Accmulate weighted scores for each model
            std::vector<float> scoresAccs (OD_NUM_OF_OBJECTS, 0.f); // [#{models} x 1]
            float wAcc = 0.f; // keep the weight accumulation (for normalization)
            
            for (int j = 0; j < nv; j++)
            {
                float w = 1.f / sqrtf(powf(positions[i][j].x,2) + powf(positions[i][j].y,2) + powf(positions[i][j].z,2));
                
                for (int m = 0; m < scores[i][j].size(); m++)
                    scoresAccs[m] += (scores[i][j][m] * (m_RecognitionStrategy == RECOGNITION_INTERVIEW_AVG ? 1 : w));
                    
                    wAcc += w;
                    }
            
            // Normalize the accumulated weighted scores
            for (int m = 0; m < scoresAccs.size(); m++)
                scoresAccs[m] /= (m_RecognitionStrategy == RECOGNITION_INTERVIEW_AVG ? nv : wAcc);
                
                // Overwrite the individual scores
                for (int j = 0; j < nv; j++) for (int m = 0; m < scoresAccs.size(); m++)
                {
                    scores[i][j][m] = scoresAccs[m];
                }
        }
        
        // Perform independently of the strategy/consenus
        
        for (int j = 0; j < nv; j++)
        {
            int maxIdx   = -1;
            float maxVal =  0;
            for (int m = 0; m < scores[i][j].size(); m++)
            {
                if (scores[i][j][m] >= m_ObjectRejections[m] && scores[i][j][m] > maxVal)
                {
                    maxIdx = m;
                    maxVal = scores[i][j][m];
                }
            }
            
            recognitions[vids[i][j]][maxIdx+1].push_back(positions[i][j]);
        }
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
        m_ObjectRejections = rhs.m_ObjectRejections;
        
        m_LeafSize = rhs.m_LeafSize;
        m_LeafSizeModel = rhs.m_LeafSizeModel;
        m_NormalRadius = rhs.m_NormalRadius;
        m_NormalRadiusModel = rhs.m_NormalRadiusModel;
        m_PfhRadius = rhs.m_PfhRadius;
        m_PfhRadiusModel = rhs.m_PfhRadiusModel;
        
        m_RecognitionStrategy = rhs.m_RecognitionStrategy;
        m_RecognitionConsensus = rhs.m_RecognitionConsensus;
        
        m_CloudjectModels = rhs.m_CloudjectModels;
        
        m_CloudjectDetections = rhs.m_CloudjectDetections;
    }
    
    return *this;
}

ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs)
{
    m_ObjectModels = rhs.getInputObjectModels();
    setInputObjectModels(m_ObjectModels);
    m_ObjectRejections = rhs.getInputObjectRejections();
    
    m_LeafSize = rhs.getCloudjectsLeafSize();
    m_LeafSizeModel = rhs.getCloudjectModelsLeafSize();
    m_NormalRadius = rhs.getCloudjectsNormalRadius();
    m_NormalRadiusModel = rhs.getCloudjectModelsNormalRadius();
    m_PfhRadius = rhs.getCloudjectsPfhRadius();
    m_PfhRadiusModel = rhs.getCloudjectModelsPfhRadius();
    
    m_PointScoreRejectionThreshold = rhs.getPointScoreRejectionThreshold();
    
    m_RecognitionStrategy = rhs.getRecognitionStrategy();
    m_RecognitionConsensus = rhs.getRecognitionConsensus();
    
    return *this;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setInputObjectModels(vector<ObjectModel<pcl::PointXYZRGB>::Ptr> models)
{
    m_ObjectModels = models;
    
    m_CloudjectModels.resize(models.size());
    for (int m = 0; m < models.size(); m++)
    {
        int oid = models[m]->getID();
        string name = models[m]->getName();
        vector<ColorPointCloud::Ptr> views = models[m]->getViews();
        
        LFCloudjectModel<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr pCloudjectModel (new LFCloudjectModel<pcl::PointXYZRGB, pcl::PFHRGBSignature250>(oid, name, views));
        m_CloudjectModels[m] = pCloudjectModel;
    }
}

vector<ObjectModel<pcl::PointXYZRGB>::Ptr> ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getInputObjectModels() const
{
    return m_ObjectModels;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setInputObjectRejections(vector<float> rejections)
{
    m_ObjectRejections = rejections;
}

vector<float> ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getInputObjectRejections() const
{
    return m_ObjectRejections;
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

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setPointScoreRejectionThreshold(float t)
{
    m_PointScoreRejectionThreshold = t;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getPointScoreRejectionThreshold() const
{
    return m_PointScoreRejectionThreshold;
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::create()
{
    for (int m = 0; m < m_CloudjectModels.size(); m++)
    {
        m_CloudjectModels[m]->describe(m_NormalRadiusModel, m_PfhRadiusModel, m_LeafSizeModel);
        m_CloudjectModels[m]->setPointScoreRejectionThreshold(m_PointScoreRejectionThreshold);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections)
{
    m_CloudjectDetections.resize(detections.size());
    for (int i = 0; i < detections.size(); i++)
    {
        LFCloudject<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr pCloudject ( new LFCloudject<pcl::PointXYZRGB, pcl::PFHRGBSignature250>( detections[i]) );
        m_CloudjectDetections[i] = pCloudject;
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

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::setRecognitionConsensus(int consensus)
{
    m_RecognitionConsensus = consensus;
}

int ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getRecognitionConsensus() const
{
    return m_RecognitionConsensus;
}

float ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::interviewConsensus(std::vector<pcl::PointXYZ> positions, std::vector<float> values)
{
    if (m_RecognitionStrategy == 0)
    {
        float valuesAcc = 0.f;
        
        for (int v = 0; v < values.size(); v++)
            valuesAcc += values[v];
        
        return valuesAcc / values.size();
    }
    else
    {
        float wtValuesAcc = 0.f;
        float weightsAcc = 0.f;
        
        for (int v = 0; v < values.size(); v++)
        {
            float d = sqrtf(powf(positions[v].x,2) + powf(positions[v].y,2) + powf(positions[v].z,2));
            wtValuesAcc += (1.0 / d) * values[v];
            weightsAcc += (1.0 / d);
        }
        
        return wtValuesAcc / weightsAcc;
    }
}

//void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getScores(vector<vector<pcl::PointXYZ> >& positions, vector<vector<float> >& scores)
//{
//    positions.clear();
//    scores.clear();
//    
//    positions.resize(m_CloudjectDetections.size(), std::vector<pcl::PointXYZ>(NUM_OF_VIEWS));
//    scores.resize(m_CloudjectDetections.size(), std::vector<float>(NUM_OF_VIEWS * m_CloudjectModels.size()));
//    
//    for (int i = 0; i < m_CloudjectDetections.size(); i++)
//    {
//        m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius, m_LeafSize);
//        
//        // Build "positions"
//        vector<int> detectionViews = m_CloudjectDetections[i]->getViewIDs();
//        vector<pcl::PointXYZ> detectionPositions = m_CloudjectDetections[i]->getPositions();
//        
//        for (int j = 0; j < detectionViews.size(); j++)
//            positions[i][detectionViews[j]] = detectionPositions[j];
//        
//        // Build "scores"
//        for (int m = 0; m < m_CloudjectModels.size(); m++)
//        {
//            std::vector<float> detectionScores;
//            m_CloudjectModels[m]->getScore(m_CloudjectDetections[i], detectionScores);
//            
//            for (int j = 0; j < detectionScores.size(); j++)
//                scores[i][detectionViews[j] * m_CloudjectModels.size() + m] = detectionScores[j];
//        }
//    }
//}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::getScores(vector<vector<int> >& vids, vector<vector<pcl::PointXYZ> >& positions, vector<vector<vector<float> > >& scores)
{
    {
        vids.clear();
        positions.clear();
        scores.clear();
        
        for (int i = 0; i < m_CloudjectDetections.size(); i++)
        {
            m_CloudjectDetections[i]->downsample(m_LeafSize);
            m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius);
            
            vector<int> detectionViews = m_CloudjectDetections[i]->getViewIDs();
            
            // views
            vids.push_back(detectionViews);
            
            // positions
            positions.push_back(m_CloudjectDetections[i]->getPositions());
            
            // scores
            vector<vector<float> > detectionScores (detectionViews.size(), std::vector<float>(m_CloudjectModels.size()));
            for (int m = 0; m < m_CloudjectModels.size(); m++)
            {
                std::vector<float> detectionModelScores;
                m_CloudjectModels[m]->getScores(m_CloudjectDetections[i], detectionModelScores);
                
                for (int v = 0; v < detectionModelScores.size(); v++)
                    detectionScores[v][m] = detectionModelScores[v];
            }
            scores.push_back(detectionScores);
        }
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
    // Recognize the initialized cloudjects
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        m_CloudjectDetections[i]->downsample(m_LeafSize);
        m_CloudjectDetections[i]->describe(m_NormalRadius, m_PfhRadius);
        
        int maxScoreIdx;
        float maxScore = 0;
        
        for (int m = 0; m < m_CloudjectModels.size(); m++)
        {
            std::vector<float> scores;
            m_CloudjectModels[m]->getScores(m_CloudjectDetections[i], scores);
            
            float consensusScore = interviewConsensus(m_CloudjectDetections[i]->getPositions(), scores);
            if (consensusScore >= maxScore)
            {
                maxScoreIdx = m;
                maxScore    = consensusScore;
            }
        }
        
        m_CloudjectDetections[i]->setID(m_CloudjectModels[maxScoreIdx]->getID());
        m_CloudjectDetections[i]->setName(m_CloudjectModels[maxScoreIdx]->getName());
    }
    
    // Cloudjects to output format
    
    recognitions.clear();
    
    recognitions.resize(NUM_OF_VIEWS);
    for (int v = 0; v < recognitions.size(); v++)
        recognitions[v].resize(OD_NUM_OF_OBJECTS + 1);
    
    for (int i = 0; i < m_CloudjectDetections.size(); i++)
    {
        int id = m_CloudjectDetections[i]->getID();
        vector<int> viewIDs = m_CloudjectDetections[i]->getViewIDs();
        vector<pcl::PointXYZ> positions = m_CloudjectDetections[i]->getPositions();
        
        for (int v = 0; v < viewIDs.size(); v++)
            recognitions[viewIDs[v]][id].push_back(positions[v]);
    }
}

void ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::recognize(std::vector<std::vector< int > > vids, std::vector<std::vector< pcl::PointXYZ > > positions, std::vector<std::vector< std::vector<float> > > scores, vector<vector<vector<pcl::PointXYZ> > >& recognitions)
{
    recognitions.clear();
    recognitions.resize(NUM_OF_VIEWS, std::vector<std::vector<pcl::PointXYZ> >(OD_NUM_OF_OBJECTS + 1));
    
    int ni = vids.size(); // num of instances
    for (int i = 0; i < ni; i++)
    {
        int nv = vids[i].size(); // num of views
        
        // If multiocular consensus, first consensuate the scores and overwrite the individual scores
        
        if (m_RecognitionStrategy == RECOGNITION_MULTIOCULAR)
        {
            // Accmulate weighted scores for each model
            std::vector<float> scoresAccs (OD_NUM_OF_OBJECTS, 0.f); // [#{models} x 1]
            float wAcc = 0.f; // keep the weight accumulation (for normalization)
            
            for (int j = 0; j < nv; j++)
            {
                float w = 1.f / sqrtf(powf(positions[i][j].x,2) + powf(positions[i][j].y,2) + powf(positions[i][j].z,2));
                
                for (int m = 0; m < scores[i][j].size(); m++)
                    scoresAccs[m] += (scores[i][j][m] * (m_RecognitionStrategy == RECOGNITION_INTERVIEW_AVG ? 1 : w));
                
                wAcc += w;
            }
            
            // Normalize the accumulated weighted scores
            for (int m = 0; m < scoresAccs.size(); m++)
                scoresAccs[m] /= (m_RecognitionStrategy == RECOGNITION_INTERVIEW_AVG ? nv : wAcc);
            
            // Overwrite the individual scores
            for (int j = 0; j < nv; j++) for (int m = 0; m < scoresAccs.size(); m++)
            {
                scores[i][j][m] = scoresAccs[m];
            }
        }
        
        // Perform independently of the strategy/consenus
        
        for (int j = 0; j < nv; j++)
        {
            int maxIdx   = -1;
            float maxVal =  0;
            for (int m = 0; m < scores[i][j].size(); m++)
            {
                if (scores[i][j][m] >= m_ObjectRejections[m] && scores[i][j][m] > maxVal)
                {
                    maxIdx = m;
                    maxVal = scores[i][j][m];
                }
            }
            
            recognitions[vids[i][j]][maxIdx+1].push_back(positions[i][j]);
        }
    }
}