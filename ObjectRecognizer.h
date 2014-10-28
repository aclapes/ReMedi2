//
//  ObjectRecognizer.h
//  remedi2
//
//  Created by Albert Clapés on 16/09/14.
//
//

#ifndef __remedi2__ObjectRecognizer__
#define __remedi2__ObjectRecognizer__

#include <iostream>

#include "ObjectRecognizer.h"
#include "ObjectModel.hpp"
#include "CloudjectModel.hpp"
#include "constants.h"

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

template<typename PointT, typename SignatureT>
class ObjectRecognizer
{
};

template<>
class ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>
{
    
    typedef pcl::PointXYZRGB ColorPointT;
    typedef pcl::PointCloud<ColorPointT> ColorPointCloud;
    typedef pcl::FPFHSignature33 FPFHSignatureT;
    
public:
    ObjectRecognizer();
    ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs);
    ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs);
    ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs);
    ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs);
    
    void setInputObjectModels(vector<ObjectModel<ColorPointT>::Ptr> models);
    vector<ObjectModel<ColorPointT>::Ptr> getInputObjectModels() const;
    void setInputObjectRejections(vector<float> rejections);
    vector<float> getInputObjectRejections() const;

    void setCloudjectsLeafSize(float leafSize);
    void setCloudjectModelsLeafSize(float leafSize);
    void setCloudjectsNormalRadius(float r);
    void setCloudjectModelsNormalRadius(float r);
    void setCloudjectsPfhRadius(float r);
    void setCloudjectModelsPfhRadius(float r);
    float getCloudjectsLeafSize() const;
    float getCloudjectModelsLeafSize() const;
    float getCloudjectsNormalRadius() const;
    float getCloudjectModelsNormalRadius() const;
    float getCloudjectsPfhRadius() const;
    float getCloudjectModelsPfhRadius() const;
    
    void setPointScoreRejectionThreshold(float t);
    float getPointScoreRejectionThreshold() const;
    
    void create();
    
    void setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections);
    
    void setRecognitionStrategy(int strategy);
    int getRecognitionStrategy() const;
    
    void getScores(vector<vector<int> >& vids, vector<vector<pcl::PointXYZ> >& positions, vector<vector<vector<float> > >& scores);
    void recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions);
    void recognize(std::vector<std::vector< int > > vids,
                   std::vector<std::vector< pcl::PointXYZ > > positions,
                   std::vector<std::vector< std::vector<float> > > scores,
                   vector<vector<vector<pcl::PointXYZ> > >& recognitions);
    
private:
    float interviewConsensus(std::vector<pcl::PointXYZ> positions, std::vector<float> values);

    vector<ObjectModel<ColorPointT>::Ptr> m_ObjectModels;
    vector<LFCloudjectModel<ColorPointT,FPFHSignatureT>::Ptr> m_CloudjectModels;
    vector<float> m_ObjectRejections;
    
    float m_LeafSize;
    float m_LeafSizeModel;
    float m_NormalRadius;
    float m_NormalRadiusModel;
    float m_PfhRadius;
    float m_PfhRadiusModel;
    
    float m_PointScoreRejectionThreshold;
    
    int m_RecognitionStrategy;
    
    vector<LFCloudject<ColorPointT,FPFHSignatureT>::Ptr> m_CloudjectDetections;
};

template<>
class ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>
{
    
    typedef pcl::PointXYZRGB ColorPointT;
    typedef pcl::PointCloud<ColorPointT> ColorPointCloud;
    typedef pcl::PFHRGBSignature250 PFHRGBSignatureT;
    
public:
    ObjectRecognizer();
    ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs);
    ObjectRecognizer(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs);
    ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& rhs);
    ObjectRecognizer<pcl::PointXYZRGB, pcl::PFHRGBSignature250>& operator=(const ObjectRecognizer<pcl::PointXYZRGB, pcl::FPFHSignature33>& rhs);
    
    void setInputObjectModels(vector<ObjectModel<ColorPointT>::Ptr> models);
    vector<ObjectModel<ColorPointT>::Ptr> getInputObjectModels() const;
    void setInputObjectRejections(vector<float> rejections);
    vector<float> getInputObjectRejections() const;

    void setCloudjectsLeafSize(float leafSize);
    void setCloudjectModelsLeafSize(float leafSize);
    void setCloudjectsNormalRadius(float r);
    void setCloudjectModelsNormalRadius(float r);
    void setCloudjectsPfhRadius(float r);
    void setCloudjectModelsPfhRadius(float r);
    float getCloudjectsLeafSize() const;
    float getCloudjectModelsLeafSize() const;
    float getCloudjectsNormalRadius() const;
    float getCloudjectModelsNormalRadius() const;
    float getCloudjectsPfhRadius() const;
    float getCloudjectModelsPfhRadius() const;
    
    void setPointScoreRejectionThreshold(float t);
    float getPointScoreRejectionThreshold() const;
    
    void create();
    
    void setInputDetections(vector<vector<pair<int,ColorPointCloud::Ptr> > > detections);
    
    void setRecognitionStrategy(int strategy);
    int getRecognitionStrategy() const;

    void getScores(vector<vector<int> >& vids, vector<vector<pcl::PointXYZ> >& positions, vector<vector<vector<float> > >& scores);
    void recognize(vector<vector<vector<pcl::PointXYZ> > >& recognitions);
    void recognize(std::vector<std::vector< int > > vids,
                   std::vector<std::vector< pcl::PointXYZ > > positions,
                   std::vector<std::vector< std::vector<float> > > scores,
                   vector<vector<vector<pcl::PointXYZ> > >& recognitions);
    
private:
    float interviewConsensus(std::vector<pcl::PointXYZ> positions, std::vector<float> values);

    vector<ObjectModel<ColorPointT>::Ptr> m_ObjectModels;
    vector<LFCloudjectModel<ColorPointT,PFHRGBSignatureT>::Ptr> m_CloudjectModels;
    vector<float> m_ObjectRejections;
    
    float m_LeafSize;
    float m_LeafSizeModel;
    float m_NormalRadius;
    float m_NormalRadiusModel;
    float m_PfhRadius;
    float m_PfhRadiusModel;
    
    float m_PointScoreRejectionThreshold;
    
    int m_RecognitionStrategy;

    vector<LFCloudject<ColorPointT,PFHRGBSignatureT>::Ptr> m_CloudjectDetections;
};

#endif /* defined(__remedi2__ObjectRecognizer__) */
