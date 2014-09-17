#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>

#include "Cloudject.hpp"

#include <vector>

using namespace std;

template<typename PointT>
class CloudjectModelBase
{
    
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

public:
	CloudjectModelBase() {}

	CloudjectModelBase(int ID, string name, float leafSize = 0.0)
		: m_ID(ID), m_Name(name), m_LeafSize(leafSize)
	{
	}
    
    CloudjectModelBase(const CloudjectModelBase& rhs)
	{
        *this = rhs;
	}
    
    CloudjectModelBase& operator=(const CloudjectModelBase& rhs)
	{
        if (this != &rhs)
        {
            m_ID = rhs.m_ID;
            m_Name = rhs.m_Name;
            m_LeafSize = rhs.m_LeafSize;
            m_ViewClouds = rhs.m_ViewClouds;
            m_ViewCentroids = rhs.m_ViewCentroids;
            m_MedianDistsToViewCentroids = rhs.m_MedianDistsToViewCentroids;
        }
        
        return *this;
	}

	int getID() { return m_ID; }
    
    string getName() { return m_Name; }
    
    int getNumOfViews() { return m_ViewClouds.size(); }

	void addView(PointCloudPtr pView)
	{
        // Add the view itself
        
		if (m_LeafSize == 0.f)
        {
            PointCloudPtr pViewCpy (new PointCloud);
			*pViewCpy = *pView;
            
			m_ViewClouds.push_back(pViewCpy);
        }
        else
		{
			PointCloudPtr pViewFiltered (new PointCloud);
			
			pcl::ApproximateVoxelGrid<PointT> avg;
			avg.setInputCloud(pView);
			avg.setLeafSize(m_LeafSize, m_LeafSize, m_LeafSize);
			avg.filter(*pViewFiltered);

			m_ViewClouds.push_back(pViewFiltered);
		}
        
        // Add a the cloud's pre-computed centroid position
        
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*pView, centroid);

        PointT p;
        p.getVector4fMap() = centroid;
		m_ViewCentroids.push_back( p );

        // Add a the cloud's pre-computed median distance to centroid
        
		float medianDist = medianDistanceToCentroid(pView, m_ViewCentroids.back());
		m_MedianDistsToViewCentroids.push_back(medianDist);
	}
    
    void setViews(vector<PointCloudPtr> views)
    {
        for (int v = 0; v < views.size(); v++)
        {
            addView(views[v]);
        }
    }
    
    PointCloudPtr getView(int i) { return m_ViewClouds[i]; }
    
	float euclideanDistance(PointT p1, PointT p2)
	{
		return sqrt(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) + powf(p1.z - p2.z, 2));
	}

	float medianDistanceToCentroid(PointCloudPtr pCloud, PointT centroid)
	{
		vector<float> distances;

		distances.push_back(euclideanDistance(centroid, pCloud->points[0]));

		for (int i = 1; i < pCloud->points.size(); i++)
		{
			float dist = euclideanDistance(centroid, pCloud->points[i]);
			bool inserted = false;
			for (int j = 0; j < distances.size() && !inserted; j++)
			{
				if (dist < distances[j])
				{
					distances.insert(distances.begin() + j, dist);
					inserted = true;
				}
			}
		}

        int medianIdx = distances.size() / 2;
		return distances[medianIdx];
	}


	// Returns the average number of points among the views of the model
	float averageNumOfPointsInModels()
	{
		float acc = 0.f;

		for (int i = 0; i < m_ViewClouds.size(); i++)
			acc += m_ViewClouds[i]->points.size();

		return acc / m_ViewClouds.size();
	}


	float averageMedianDistanceToCentroids()
	{
		float acc = 0.f;

		for (int i = 0; i < m_MedianDistsToViewCentroids.size(); i++)
			acc += m_MedianDistsToViewCentroids[i];

		return acc / m_MedianDistsToViewCentroids.size();
	}

protected:

	vector<PointCloudPtr> m_ViewClouds;
	vector<PointT> m_ViewCentroids;
	vector<float> m_MedianDistsToViewCentroids;

	float m_LeafSize; // in case of downsampling

private:
	
	int m_ID;
    string m_Name;
};


template<typename PointT>
class LFCloudjectModelBase : public CloudjectModelBase<PointT>
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

public:
	LFCloudjectModelBase(void) 
		: CloudjectModelBase<PointT>() {}

	LFCloudjectModelBase(int ID, string name, float leafSize = 0.0, int penalty = 2, float pointRejectionThresh = 1.0, float ratioRejectionThresh = 1.0, float sigmaPenaltyThresh = 0.1)
		: CloudjectModelBase<PointT>(ID, name, leafSize),
		  m_Penalty(penalty), m_PointRejectionThresh(pointRejectionThresh), m_RatioRejectionThresh(ratioRejectionThresh), m_SigmaPenaltyThresh(sigmaPenaltyThresh)
	{}
    
    LFCloudjectModelBase(const LFCloudjectModelBase& rhs)
        : CloudjectModelBase<PointT>(rhs)
    {
        *this = rhs;
    }
    
    LFCloudjectModelBase& operator=(const LFCloudjectModelBase& rhs)
    {
        if (this != &rhs)
        {
            m_Penalty = rhs.m_Penalty;
            
            m_PointRejectionThresh = rhs.m_PointRejectionThresh;
            m_RatioRejectionThresh = rhs.m_RatioRejectionThresh;
            m_SigmaPenaltyThresh = rhs.m_SigmaPenaltyThresh;
        }
        
        return *this;
    }

//    void addViewDescriptor(DescriptorPtr pDescriptor)
//    {
//        m_ViewsDescriptors.push_back(pDescriptor);
//    }
//    
//    DescriptorPtr getViewDescriptor(int i)
//    {
//        return m_ViewsDescriptors[i];
//    }

    template<typename SignatureT>
    float getScore(vector<typename pcl::PointCloud<SignatureT>::Ptr> viewsDescriptors, typename LFCloudject<PointT,SignatureT>::Ptr pCloudject)
    {
        float penalizedScoresAcc = 0.f;
        
        for (int i = 0; i < pCloudject->getNumOfViews(); i++)
        {
            float score = matchView(pCloudject->getDescription(i));
            
            float penalty = 1.f;
			if (getPenalty() == 0)
			{
                float avg = CloudjectModelBase<PointT>::averageNumOfPointsInModels();
				float ratio = pCloudject->getNumOfPointsInView(i) / avg;
				float x = (ratio <= 1) ? ratio : 1 + (1 - (1 / ratio));
                
				penalty *= (1.0 / (m_SigmaPenaltyThresh * sqrtf(2.f * 3.14159))) * expf(-0.5f * powf((x-1)/m_SigmaPenaltyThresh, 2));
			}
			else if (getPenalty() == 1)
			{
                float avg = CloudjectModelBase<PointT>::averageMedianDistanceToCentroids();
				float diff = (pCloudject->medianDistToCentroidInView(i) - avg);

				penalty *= (1.f / (m_SigmaPenaltyThresh * sqrtf(2.f * 3.14159))) * expf(-0.5f * powf(diff/m_SigmaPenaltyThresh, 2));
			}
            
            penalizedScoresAcc += (score * penalty);
        }
        
        return penalizedScoresAcc / pCloudject->getNumOfViews();
    }

protected:
	// Returns the score of matching a description of a certain cloudject's view against the model views' descriptions
    template<typename SignatureT>
	float matchView(vector<typename pcl::PointCloud<SignatureT>::Ptr> viewsDescriptors, typename pcl::PointCloud<SignatureT>::Ptr descriptor)
	{
		// Auxiliary structures: to not match against a model point more than once

		vector<int> numOfMatches;
		numOfMatches.resize(viewsDescriptors.size(), 0);

		vector<vector<bool> > matches;

		matches.resize(viewsDescriptors.size());
		for (int i = 0; i < matches.size(); i++)
			matches[i].resize(viewsDescriptors[i]->points.size(), false);

		// Match

		float accDistToSig = 0;

		float minDistToP, ndMinDist, dist; // inner-loop vars
		int minIdxV = -1, minIdxP = -1;
		int numOfTotalMatches = 0;

		for (int p = 0; p < descriptor->points.size(); p++)
		{
			bool freeCorrespondences = false; // there is any point to match against in the views of the model?

			minDistToP = numeric_limits<float>::infinity(); // min distance to other point histogram
			ndMinDist =  numeric_limits<float>::infinity();
		
			for (int i = 0; i < viewsDescriptors.size() && numOfMatches[i] < viewsDescriptors[i]->points.size(); i++)
			{
				for (int j = 0; j < viewsDescriptors[i]->points.size(); j++)
				{
					if ( freeCorrespondences = !(matches[i][j]) ) // A point in a view can only be matched one time against
					{
						dist = battacharyyaDistanceSignatures( descriptor->points[p], viewsDescriptors[i]->points[j]/*, minDistToP*/);

						if (dist < minDistToP) // not matched yet and minimum
						{
							minDistToP = dist;
							minIdxV = i;
							minIdxP = j;
						}
						else if (dist < ndMinDist)
						{
							ndMinDist = dist;
						}
					}
				}
			}
			
			// if it is not "true", minDist is infinity. Not to accumulate infinity :S
			// And dealing with another kinds of errors
			//if ( fereCorrespondences && !(minIdx < 0 || minIdxP < 0) )
			if (minDistToP <= m_PointRejectionThresh/* && (minDistToP/ndMinDist) < m_RatioRejectionThresh*/)
			{
				accDistToSig += minDistToP;
				numOfTotalMatches ++;
				
				numOfMatches[minIdxV] ++; // aux var: easy way to know when all the points in a model have been matched
				matches[minIdxV][minIdxP] = true; // aux var: to know when a certain point in a certian model have already matched
			}
		}

		// Normalization: to deal with partial occlusions
		//float factor = (descriptor->points.size() / (float) averageNumOfPointsInModels());
		
		float avgDist = accDistToSig / numOfTotalMatches;
		float score =  1 - avgDist;

		return score; // / descriptor->points.size());
	}


	// Returns the battacharyya distance between two fpfh signatures, which are actually histograms.
	// This is a normalized [0,1] distance
	float battacharyyaDistanceSignatures(float* h1, float* h2, int length)
	{
		float accSqProd = 0.f;
		float accS1 = 0.f;
		float accS2 = 0.f;
        
		for (int b = 0.f; b < length; b++)
		{
			accSqProd += sqrt(h1[b] * h2[b]);
			accS1 += h1[b];
			accS2 += h2[b];
		}

		float f = 1.f / sqrt((accS1/length) * (accS2/length) * (length*length));

		return sqrt(1.f - f * accSqProd);
	}


	// Returns the euclidean distance between two fpfh signatures, which are actually histograms
	float euclideanDistanceSignatures(float* h1, float* h2, int length)
	{
		float acc = 0.f;
		for (int b = 0; b < length; b++)
		{
			acc += powf(h1[b] - h2[b], 2.0);
		}

		return sqrtf(acc);
	}


	// Returns the euclidean distance between two fpfh signatures, which are actually histograms
	// subtracting bin-by-bin while the square root of the accumulated subtractions are lower than
	// a threshold. Otherwise, return the threshold.
	float euclideanDistanceSignatures(float* h1, float* h2, int length ,float thresh)
	{
		float acc = 0;
		for (int b = 0; b < length; b++)
		{
			if (sqrtf(acc) >= thresh)
				return thresh;

			acc += powf(h1[b] - h2[b], 2.0);
		}

		return sqrtf(acc);
	}

	int getPenalty()
	{
		return m_Penalty;
	}
    
	//
	// Protected members
	// 

//	// The descriptions of the different views
//	vector<DescriptorPtr>		m_ViewsDescriptors;
	// A valid best correspondence should be a distance below it (experimentally selected)
	float       m_PointRejectionThresh;
	float       m_RatioRejectionThresh;
	float		m_SigmaPenaltyThresh;

	int			m_Penalty;
	enum		Penalty { None, NumOfPoints, MedianDistToCentroid };
};


//
// Templates
//

// Generic template

template<typename PointT>
class LFCloudjectModel : public LFCloudjectModelBase<PointT>
{};


// Partially specialized template

template<>
class LFCloudjectModel<pcl::PointXYZRGB> : public LFCloudjectModelBase<pcl::PointXYZRGB>
{
    typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
	typedef typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
    typedef pcl::search::KdTree<pcl::PointXYZRGB> KdTree;
    typedef typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr KdTreePtr;
public:
    
	LFCloudjectModel(int ID, string name, float leafSize = 0.0, int penalty = 1, float pointRejectionThresh = 1.0, float ratioRejectionThresh = 1.0, float sigmaPenaltyThresh = 0.1)
    : LFCloudjectModelBase<pcl::PointXYZRGB>(ID, name, leafSize, penalty, pointRejectionThresh, ratioRejectionThresh, sigmaPenaltyThresh), m_DescriptionType(0)
	{}
    
    LFCloudjectModel(const LFCloudjectModel& rhs)
    : LFCloudjectModelBase<pcl::PointXYZRGB>(rhs)
    {
        *this = rhs;
    }
    
    //	virtual ~LFCloudjectModel() {}
    
    LFCloudjectModel& operator=(const LFCloudjectModel& rhs)
    {
        if (this != &rhs)
        {
            m_DescriptionType = rhs.m_DescriptionType;
            m_NormalRadius = rhs.m_NormalRadius;
            m_PfhRadius = rhs.m_PfhRadius;
            m_ViewsDescriptionsFPFH = rhs.m_ViewsDescriptionsFPFH;
            m_ViewsDescriptionsPFHRGB = rhs.m_ViewsDescriptionsPFHRGB;
            
        }
        return *this;
    }
    
    void setDescriptionType(int type)
    {
        m_DescriptionType = type;
    }
    
    void setNormalRadius(float r)
    {
        m_NormalRadius = r;
    }
    
    void setPfhRadius(float r)
    {
        m_PfhRadius = r;
    }
    
	// Describe all the model views
	void describe(float leafSize = 0.f)
	{
		for (int i = 0; i < LFCloudjectModelBase<PointT>::getNumOfViews(); i++)
		{
            PointCloudPtr view = LFCloudjectModelBase<PointT>::getView(i);
            
            if (m_DescriptionType == DESCRIPTION_FPFH)
            {
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr pDescriptor (new pcl::PointCloud<pcl::FPFHSignature33>);
                describeViewFPFH(view, m_NormalRadius, m_PfhRadius, *pDescriptor, leafSize);
                m_ViewsDescriptionsFPFH.push_back(pDescriptor);
            }
            else if (m_DescriptionType == DESCRIPTION_PFHRGB)
            {
                pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr pDescriptor (new pcl::PointCloud<pcl::PFHRGBSignature250>);
                describeViewPFHRGB(view, m_NormalRadius, m_PfhRadius, *pDescriptor, leafSize);
                m_ViewsDescriptionsPFHRGB.push_back(pDescriptor);
            }
		}
	}
    
    float getScore(typename LFCloudject<PointT,pcl::PointCloud<pcl::FPFHSignature33> >::Ptr pCloudject)
    {
        return LFCloudjectBase<PointT>::getScore<pcl::PointCloud<pcl::FPFHSignature33>(m_ViewsDescriptorsFPFH, pCloudject);
    }
    
    float getScore(typename LFCloudject<PointT,pcl::PointCloud<pcl::PFHRGBSignature250> >::Ptr pCloudject)
    {
        return LFCloudjectBase<PointT>::getScore<pcl::PFHRGBSignature250>(m_ViewsDescriptorsPFHRGB, pCloudject);
    }
    
    enum { DESCRIPTION_FPFH, DESCRIPTION_PFHRGB };
    
    typedef boost::shared_ptr<LFCloudjectModel<PointT> > Ptr;
    
private:
    
	// Compute the description of a view, actually
	void describeViewFPFH(PointCloudPtr pView,
                          float normalRadius, float fpfhRadius,
                          pcl::PointCloud<pcl::FPFHSignature33>& descriptor, float leafSize = 0.f)
	{
        // Compute the description of a view, performing
        // a prior downsampling to speed up the process
        if (leafSize > LFCloudjectModelBase<PointT>::m_LeafSize)
        {
            PointCloudPtr pViewF (new PointCloud);
            
            pcl::VoxelGrid<PointT> avg;
            avg.setInputCloud(pView);
            avg.setLeafSize(leafSize, leafSize, leafSize);
            avg.filter(*pViewF);
            
            pViewF.swap(pView);
        }
        
		//
		// Normals preprocess
		//
        
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (pView);
        
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		KdTreePtr tree (new KdTree);
		ne.setSearchMethod (tree);
        
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);
        
		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normalRadius);
        
		// Compute the features
		ne.compute (*pNormals);
        
		//
		// FPFH description extraction
		//
        
		pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (pView);
		fpfh.setInputNormals (pNormals);
        
		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		tree = KdTreePtr(new KdTree);
		fpfh.setSearchMethod (tree);
        
		// Output datasets
		// * initialize outside
        
		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (fpfhRadius);
        
		// Compute the features
		fpfh.compute (descriptor);
	}
    
    // Compute the description of a view, actually
	void describeViewPFHRGB(PointCloudPtr pView, float normalRadius, float pfhrgbRadius,
                            pcl::PointCloud<pcl::PFHRGBSignature250>& descriptor, float leafSize = 0.f)
	{
        // Compute the description of a view, performing
        // a prior downsampling to speed up the process
        if (leafSize > LFCloudjectModelBase<PointT>::m_LeafSize)
        {
            PointCloudPtr pViewF (new PointCloud);
            
            pcl::VoxelGrid<PointT> avg;
            avg.setInputCloud(pView);
            avg.setLeafSize(leafSize, leafSize, leafSize);
            avg.filter(*pViewF);
            
            pViewF.swap(pView);
        }
        
		//
		// Normals preprocess
		//
        
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (pView);
        
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		KdTreePtr tree (new KdTree);
		ne.setSearchMethod (tree);
        
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);
        
		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normalRadius);
        
		// Compute the features
		ne.compute (*pNormals);
        
		//
		// FPFH description extraction
		//
        
        pcl::PFHRGBEstimation<PointT,pcl::Normal,pcl::PFHRGBSignature250> pfhrgb;
		pfhrgb.setInputCloud (pView);
		pfhrgb.setInputNormals (pNormals);
        
		// Create an empty kdtree representation, and pass it to the PFHRGB estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		tree = KdTreePtr(new KdTree);
		pfhrgb.setSearchMethod (tree);
        
		// Output datasets
		// * initialize outside
        
		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		pfhrgb.setRadiusSearch (pfhrgbRadius);
        
		// Compute the features
		pfhrgb.compute (descriptor);
	}
    
    //
    // Attributes
    //
    
    int m_DescriptionType;
    
    // <X>PFH
    float m_NormalRadius;
    float m_PfhRadius;
    
    // PFHRGB
    vector<pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr> m_ViewsDescriptionsPFHRGB;
    
    // FPFH
    vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> m_ViewsDescriptionsFPFH;
    
};

// Partially specialized template

template<>
class LFCloudjectModel<pcl::PointXYZ> : public LFCloudjectModelBase<pcl::PointXYZ>
{
    typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
    typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
    typedef typename pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;
public:
    
	LFCloudjectModel(int ID, string name, float leafSize = 0.0, int penalty = 1, float pointRejectionThresh = 1.0, float ratioRejectionThresh = 1.0, float sigmaPenaltyThresh = 0.1)
    : LFCloudjectModelBase<PointT>(ID, name, leafSize, penalty, pointRejectionThresh, ratioRejectionThresh, sigmaPenaltyThresh), m_DescriptionType(0)
	{}
    
    LFCloudjectModel(const LFCloudjectModel& rhs)
    : LFCloudjectModelBase<PointT>(rhs)
    {
        *this = rhs;
    }
    
    //	virtual ~LFCloudjectModel() {}
    
    LFCloudjectModel& operator=(const LFCloudjectModel& rhs)
    {
        if (this != &rhs)
        {
            m_DescriptionType = rhs.m_DescriptionType;
            m_ViewsDescriptionsFPFH = rhs.m_ViewsDescriptionsFPFH;
        }
        return *this;
    }
    
    void setDescriptionType(int type)
    {
        m_DescriptionType = type;
    }
    
    void setNormalRadius(float r)
    {
        m_NormalRadius = r;
    }
    
    void setPfhRadius(float r)
    {
        m_PfhRadius = r;
    }
    
	// Describe all the model views
	void describe(float leafSize = 0.f)
	{
        m_ViewsDescriptionsFPFH.resize(LFCloudjectModelBase<PointT>::getNumOfViews());
		for (int i = 0; i < LFCloudjectModelBase<PointT>::getNumOfViews(); i++)
		{
            PointCloudPtr view = LFCloudjectModelBase<PointT>::getView(i);
            
            if (m_DescriptionType == DESCRIPTION_FPFH)
            {
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr pDescriptor (new pcl::PointCloud<pcl::FPFHSignature33>);
                describeViewFPFH(view, m_NormalRadius, m_PfhRadius, *pDescriptor, leafSize);
                m_ViewsDescriptionsFPFH[i] = pDescriptor;
            }
		}
	}
    
    float getScore(typename LFCloudject<PointT, pcl::PointCloud<pcl::FPFHSignature33> >::Ptr pCloudject)
    {
        return LFCloudjectBase<PointT>::getScore<pcl::PointCloud<pcl::FPFHSignature33>(m_ViewsDescriptorsFPFH, pCloudject);
    }
    
    enum { DESCRIPTION_FPFH };
    
    typedef boost::shared_ptr<LFCloudjectModel<PointT> > Ptr;
    
private:
    
	// Compute the description of a view, actually
	void describeViewFPFH(PointCloudPtr pView,
                          float normalRadius, float fpfhRadius,
                          pcl::PointCloud<pcl::FPFHSignature33>& descriptor, float leafSize = 0.f)
	{
        // Compute the description of a view, performing
        // a prior downsampling to speed up the process
        if (leafSize > LFCloudjectModelBase<PointT>::m_LeafSize)
        {
            PointCloudPtr pViewF (new PointCloud);
            
            pcl::VoxelGrid<PointT> avg;
            avg.setInputCloud(pView);
            avg.setLeafSize(leafSize, leafSize, leafSize);
            avg.filter(*pViewF);
            
            pViewF.swap(pView);
        }
        
		//
		// Normals preprocess
		//
        
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<PointT, pcl::Normal> ne;
		ne.setInputCloud (pView);
        
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		KdTreePtr tree (new KdTree);
		ne.setSearchMethod (tree);
        
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);
        
		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (normalRadius);
        
		// Compute the features
		ne.compute (*pNormals);
        
		//
		// FPFH description extraction
		//
        
		pcl::FPFHEstimation<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (pView);
		fpfh.setInputNormals (pNormals);
        
		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		tree = KdTreePtr(new KdTree);
		fpfh.setSearchMethod (tree);
        
		// Output datasets
		// * initialize outside
        
		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (fpfhRadius);
        
		// Compute the features
		fpfh.compute (descriptor);
	}

    
    //
    // Attributes
    //
    
    int m_DescriptionType;
    
    // <X>PFH
    float m_NormalRadius;
    float m_PfhRadius;
    
    // FPFH
    vector<pcl::PointCloud<pcl::FPFHSignature33>::Ptr> m_ViewsDescriptionsFPFH;
    
};


// Explicit template instantation
template class CloudjectModelBase<pcl::PointXYZ>;
template class CloudjectModelBase<pcl::PointXYZRGB>;

template class LFCloudjectModelBase<pcl::PointXYZ>;
template class LFCloudjectModelBase<pcl::PointXYZRGB>;
