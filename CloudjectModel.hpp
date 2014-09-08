#pragma once

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfhrgb.h>

#include "Cloudject.hpp"

#include <vector>

using namespace std;

template<typename PointT, typename SignatureT>
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
		pcl::compute3DCentroid(*pView), centroid);
		m_ViewCentroids.push_back( PointT(centroid.x(), centroid.y(), centroid.z()) );

        // Add a the cloud's pre-computed median distance to centroid
        
		float medianDist = medianDistanceToCentroid(pView, c);
		m_MedianDistsToViewCentroids.push_back(medianDist);
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


template<typename PointT, typename SignatureT>
class LFCloudjectModelBase : public CloudjectModelBase<PointT,SignatureT>
{
	typedef typename pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
	typedef typename pcl::PointCloud<SignatureT> Descriptor;
	typedef typename pcl::PointCloud<SignatureT>::Ptr DescriptorPtr;

protected:
	LFCloudjectModelBase(void) 
		: CloudjectModelBase<PointT,SignatureT>() {}

	LFCloudjectModelBase(int ID, string name, float leafSize = 0.0, int penalty = 2, float pointRejectionThresh = 1.0, float ratioRejectionThresh = 1.0, float sigmaPenaltyThresh = 0.1)
		: CloudjectModelBase<PointT,SignatureT>(ID, name, leafSize),
		  m_Penalty(penalty), m_PointRejectionThresh(pointRejectionThresh), m_RatioRejectionThresh(ratioRejectionThresh), m_SigmaPenaltyThresh(sigmaPenaltyThresh)
	{}
    
    LFCloudjectModelBase(const LFCloudjectModelBase& rhs)
        : CloudjectModelBase<PointT,SignatureT>(rhs)
    {
        *this = rhs;
    }
    
    LFCloudjectModelBase& operator=(const LFCloudjectModelBase& rhs)
    {
        if (this != &rhs)
        {
            m_ViewsDescriptors = rhs.m_ViewsDescriptors;

            m_Penalty = rhs.m_Penalty;
            
            m_PointRejectionThresh = rhs.m_PointRejectionThresh;
            m_RatioRejectionThresh = rhs.m_RatioRejectionThresh;
            m_SigmaPenaltyThresh = rhs.m_SigmaPenaltyThresh;
        }
        
        return *this;
    }

	int getID() { return CloudjectModelBase<PointT, SignatureT>::getID(); }
    string getName() { return CloudjectModelBase<PointT, SignatureT>::getName(); }
    
    int getNumOfViews() { return CloudjectModelBase<PointT, SignatureT>::getNumOfViews(); }

	void addView(PointCloudPtr pCloud) { CloudjectModelBase<PointT,SignatureT>::addView(pCloud); }
    PointCloudPtr getView(int i) { return CloudjectModelBase<PointT,SignatureT>::getView(i); }
    
    void addViewDescriptor(DescriptorPtr pDescriptor)
    { m_ViewsDescriptors.push_back(pDescriptor); }
    DescriptorPtr getViewDescriptor(int i) { return m_ViewsDescriptors[i]; }

	float euclideanDistance(PointT p1, PointT p2) { return CloudjectModelBase<PointT,SignatureT>::euclideanDistance(p1,p2); }
	float medianDistanceToCentroid(PointCloudPtr pCloud, PointT centroid)
	{ return CloudjectModelBase<PointT,SignatureT>::medianDistanceToCentroid(pCloud, centroid); }

	float averageNumOfPointsInModels() { return CloudjectModelBase<PointT,SignatureT>::averageNumOfPointsInModels(); }
	float averageMedianDistanceToCentroids() { return CloudjectModelBase<PointT,SignatureT>::averageMedianDistanceToCentroids(); }

    float match(LFCloudject<PointT,SignatureT> c)
    {
        float penalizedScores = 0.f;
        
        for (int i = 0; i < c.getNumOfViews(); i++)
        {
            float score = matchView(c.getDescription(i));
            
            float penalty = 1.f;
			if (getPenalty() == 0)
			{
				float ratio = (((float) c.getNumOfPointsInView(i)) / averageNumOfPointsInModels());
				float x = (ratio <= 1) ? ratio : 1 + (1 - (1 / ratio));
                
				penalty *= (1.0 / (m_SigmaPenaltyThresh * sqrtf(2.f * 3.14159))) * expf(-0.5f * powf((x-1)/m_SigmaPenaltyThresh, 2));
			}
			else if (getPenalty() == 1)
			{
				float diff = (c.medianDistToCentroidInView(i) - averageMedianDistanceToCentroids());
                
				penalty *= (1.f / (m_SigmaPenaltyThresh * sqrtf(2.f * 3.14159))) * expf(-0.5f * powf(diff/m_SigmaPenaltyThresh, 2));
			}
            
            penalizedScore += (score * penalty);
        }
        
        return penalizedScore / c.getNumOfViews();
    }

//	// Returns the score of matching a cloudject against the model
//	float match(LFCloudject<PointT,SignatureT> c)
//	{
//		float sigma = m_SigmaPenaltyThresh;
//
//        if (c.getViewA()->empty() || c.getViewB()->empty())
//		{
//			float score = c.getViewB()->empty() ?
//                matchView(c.getDescriptionA()) : matchView(c.getDescriptionB());
//			float penalty = 1.f;
//			
//			if (getPenalty() == 0)
//			{
//				float ratio = c.getViewB()->empty() ? (((float) c.getNumOfPointsInViewA()) / averageNumOfPointsInModels()) : (((float) c.getNumOfPointsInViewB()) / averageNumOfPointsInModels());
//				
//				float x = (ratio <= 1) ? ratio : 1 + (1 - (1 / ratio));
////				(ratio <= 1) ? (x = ratio) : ( x = 1 + (1 - (1 / ratio)) );
//
//				penalty = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf((x-1)/sigma, 2));
//			}
//			else if (getPenalty() == 1)
//			{
//				float diff = c.getViewB()->empty() ? (c.medianDistToCentroidInViewA() - averageMedianDistanceToCentroids()) : (c.medianDistToCentroidInViewB() - averageMedianDistanceToCentroids());
//				penalty = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf(diff/sigma, 2));
//			}
//
////            c.getViewB()->empty() ? cout << "(" << score << "*" << penalty << ",)": cout << "(," << score << "*" << penalty << ")";
//			return score * penalty;
//		}
//		else
//		{
//			float scoreA = matchView(c.getDescriptionA());
//			float scoreB = matchView(c.getDescriptionB());
//
//			float penaltyA = 1.f, penaltyB = 1.f;
//
//			if (getPenalty() == 0)
//			{
//				float ratioA = ((float) c.getNumOfPointsInViewA()) / averageNumOfPointsInModels();
//				float ratioB = ((float) c.getNumOfPointsInViewB()) / averageNumOfPointsInModels();
//				float xA, xB;
//				
//				xA = (ratioA <= 1) ? ratioA : 1 + (1 - (1 / ratioA));
//				xB = (ratioB <= 1) ? ratioB : 1 + (1 - (1 / ratioB));
//
//				penaltyA = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf((xA-1)/sigma, 2));
//				penaltyB = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf((xB-1)/sigma, 2));
//			}
//			else if (getPenalty() == 1)
//			{
//				float diffA = c.medianDistToCentroidInViewA() - averageMedianDistanceToCentroids();
//				float diffB = c.medianDistToCentroidInViewB() - averageMedianDistanceToCentroids();
//
//				penaltyA = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf(diffA/sigma, 2));
//				penaltyB = (1.0 / (sigma * sqrtf(2 * 3.14159))) * expf(-0.5 * powf(diffB/sigma, 2));
//			}
//            
//			// TODO: more sophisticated fusion
//            float penalizedScoreA = scoreA * penaltyA;
//            float penalizedScoreB = scoreB * penaltyB;
////            float distanceWeightA = 1.0 / c.getPosA();
////            float distanceWeightB = c.getPosB();
//            
////            cout << "(" << scoreA << "*" << penaltyA << "," << scoreB << "*" << penaltyB << ")";
//			return (penalizedScoreA + penalizedScoreB) / 2.0;
//		}
//	}


	// Returns the score of matching a description of a certain cloudject's view against the model views' descriptions
	float matchView(DescriptorPtr descriptor)
	{
		// Auxiliary structures: to not match against a model point more than once

		vector<int> numOfMatches;
		numOfMatches.resize(m_ViewsDescriptors.size(), 0);

		vector<vector<bool> > matches;

		matches.resize(m_ViewsDescriptors.size());
		for (int i = 0; i < matches.size(); i++)
			matches[i].resize(m_ViewsDescriptors[i]->points.size(), false);

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
		
			for (int i = 0; i < m_ViewsDescriptors.size() && numOfMatches[i] < m_ViewsDescriptors[i]->points.size(); i++) 
			{
				for (int j = 0; j < m_ViewsDescriptors[i]->points.size(); j++)
				{
					if ( freeCorrespondences = !(matches[i][j]) ) // A point in a view can only be matched one time against
					{
						dist = battacharyyaDistanceSignatures( descriptor->points[p], m_ViewsDescriptors[i]->points[j]/*, minDistToP*/);

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
	float battacharyyaDistanceSignatures(SignatureT s1, SignatureT s2)
	{
		float accSqProd = 0;
		float accS1 = 0;
		float accS2 = 0;
		for (int b = 0; b < sizeof(s1.histogram) / sizeof(s1.histogram[0]); b++)
		{
			accSqProd += sqrt(s1.histogram[b] * s2.histogram[b]);
			accS1 += s1.histogram[b];
			accS2 += s2.histogram[b];
		}

		float f = 1.0 / sqrt((accS1/33) * (accS2/33) * (33*33));

		return sqrt(1 - f * accSqProd);
	}


	// Returns the euclidean distance between two fpfh signatures, which are actually histograms
	float euclideanDistanceSignatures(SignatureT s1, SignatureT s2)
	{
		float acc = 0;
		for (int b = 0; b < sizeof(s1.histogram) / sizeof(s1.histogram[0]); b++)
		{
			acc += powf(s1.histogram[b] - s2.histogram[b], 2.0);
		}

		return sqrtf(acc);
	}


	// Returns the euclidean distance between two fpfh signatures, which are actually histograms
	// subtracting bin-by-bin while the square root of the accumulated subtractions are lower than
	// a threshold. Otherwise, return the threshold.
	float euclideanDistanceSignatures(SignatureT s1, SignatureT s2, float thresh)
	{
		float acc = 0;
		for (int b = 0; b < sizeof(s1.histogram) / sizeof(s1.histogram[0]); b++)
		{
			if (sqrtf(acc) >= thresh)
				return thresh;

			acc += powf(s1.histogram[b] - s2.histogram[b], 2.0);
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

	// The descriptions of the different views
	vector<DescriptorPtr>		m_ViewsDescriptors;
	// A valid best correspondence should be a distance below it (experimentally selected)
	float							m_PointRejectionThresh;
	float							m_RatioRejectionThresh;
	float							m_SigmaPenaltyThresh;

	int								m_Penalty;
	enum							Penalty { None, NumOfPoints, MedianDistToCentroid };

private:

	float penalty(float diff)
	{ }
};


//
// Templates
//

// Generic template
template<typename PointT, typename SignatureT>
class LFCloudjectModel : public LFCloudjectModelBase<PointT,SignatureT>
{};

// Partially specialized template
template<typename PointT>
class LFCloudjectModel<PointT, pcl::FPFHSignature33> : public LFCloudjectModelBase<PointT, pcl::FPFHSignature33>
{
	typedef pcl::PointCloud<pcl::FPFHSignature33> Descriptor;
	typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr DescriptorPtr;
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::search::KdTree<PointT> KdTree;
    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;

	typedef LFCloudject<PointT,pcl::FPFHSignature33> LFCloudject;

public:

	LFCloudjectModel(int ID, string name, float leafSize = 0.0, int penalty = 1, float pointRejectionThresh = 1.0, float ratioRejectionThresh = 1.0, float sigmaPenaltyThresh = 0.1)
		: LFCloudjectModelBase<PointT,pcl::FPFHSignature33>(ID, name, leafSize, penalty,
		  pointRejectionThresh, ratioRejectionThresh, sigmaPenaltyThresh)
	{}
    
    LFCloudjectModel(const LFCloudjectModel& rhs)
        : LFCloudjectModelBase<PointT,pcl::FPFHSignature33>(rhs)
    {
        *this = rhs;
    }

//	virtual ~LFCloudjectModel() {}

    LFCloudjectModel& operator=(const LFCloudjectModel& rhs)
    {
        return *this;
    }
    
	int getID() { return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::getID(); }
    string getName() { return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::getName(); }
	
    int getNumOfViews() { return CloudjectModelBase<PointT,pcl::FPFHSignature33>::getNumOfViews(); }
	void addView(PointCloudPtr pCloud) { LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::addView(pCloud); }
    PointCloudPtr getView(int i) { return CloudjectModelBase<PointT,pcl::FPFHSignature33>::getView(i); }
    
    void addViewDescriptor(DescriptorPtr pDescriptor)
    { LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::addViewDescriptor(pDescriptor); }
    DescriptorPtr getViewDescriptor(int i) { return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::getViewDescriptor(i); }
    
	float euclideanDistance(PointT p1, PointT p2) { return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::euclideanDistance(p1,p2); }
	float medianDistanceToCentroid(PointCloudPtr pCloud, PointT centroid)
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::medianDistanceToCentroid(pCloud, centroid); }
	
	float averageNumOfPointsInModels() 
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::averageNumOfPointsInModels(); }
	float averageMedianDistanceToCentroids() 
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::averageMedianDistanceToCentroids(); }

	float match(LFCloudject c)
	{
        // DEBUG
        if (c.getDescriptionA()->empty() && c.getDescriptionB()->empty())
            cout << "null" << endl;
        return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::match(c);
    }

	int getPenalty()
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::getPenalty(); }


	// Describe all the model views
	void describe(float normalRadius, float fpfhRadius)
	{
		for (int i = 0; i < getNumOfViews(); i++)
		{
			DescriptorPtr pDescriptor (new Descriptor);
            PointCloudPtr view = getView(i);
			describeView(view, normalRadius, fpfhRadius, *pDescriptor);
			addViewDescriptor(pDescriptor);
		}
	}

private:
	float matchView(DescriptorPtr pDescriptor)
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::matchView(pDescriptor); }

	float battacharyyaDistanceSignatures(pcl::FPFHSignature33 s1, pcl::FPFHSignature33 s2)
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::battacharyyaDistanceSignatures(s1, s2); }

	float euclideanDistanceSignatures(pcl::FPFHSignature33 s1, pcl::FPFHSignature33 s2)
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::euclideanDistanceSignatures(s1, s2); }

	float euclideanDistanceSignatures(pcl::FPFHSignature33 s1, pcl::FPFHSignature33 s2, float thresh)
	{ return LFCloudjectModelBase<PointT,pcl::FPFHSignature33>::euclideanDistanceSignatures(s1, s2, thresh); }


	// Compute the description of a view, performing
	// a prior downsampling to speed up the process
	void describeView(PointCloudPtr pView, 
					  float leafSize, float normalRadius, float fpfhRadius,
					  Descriptor& descriptor)
	{
		PointCloudPtr pViewF (new PointCloud);

		pcl::ApproximateVoxelGrid<PointT> avg;
		avg.setInputCloud(pView);
		avg.setLeafSize(leafSize, leafSize, leafSize);
		avg.filter(*pViewF);

		pViewF.swap(pView);

		describeView(pView, normalRadius, fpfhRadius, descriptor);
	}


	// Compute the description of a view, actually
	void describeView(PointCloudPtr pView, 
					  float normalRadius, float fpfhRadius,
					  Descriptor& descriptor)
	{
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
};


//// Partially specialized template
//template<typename PointT>
//class LFCloudjectModel<PointT,pcl::PFHRGBSignature250> : public LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>
//{
//	typedef pcl::PointCloud<pcl::PFHRGBSignature250> Descriptor;
//	typedef pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr DescriptorPtr;
//	typedef pcl::PointCloud<PointT> PointCloud;
//	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
//    typedef pcl::search::KdTree<PointT> KdTree;
//    typedef typename pcl::search::KdTree<PointT>::Ptr KdTreePtr;
//
//	typedef LFCloudject<PointT,pcl::PFHRGBSignature250> LFCloudject;
//
//public:
//
//	LFCloudjectModel(int ID, int nViewpoints = 3, float leafSize = 0.0, float pointRejectionThresh = 1.0, 
//		float ratioRejectionThresh = 1.0, int penalty = 1, float sigmaPenaltyThresh = 0.1)
//		: LFCloudjectModelBase<PointT, pcl::PFHRGBSignature250>(ID, nViewpoints, leafSize,
//		  pointRejectionThresh, ratioRejectionThresh, penalty, sigmaPenaltyThresh)
//	{}
//
//	virtual ~LFCloudjectModel() {}
//
//	int getID() { return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::getID(); }
//    string getName() { return LFCloudjectModelBase<PointT, pcl::PFHRGBSignature250>::getName(); }
//    int getNumOfViews() { return CloudjectModelBase<PointT,pcl::PFHRGBSignature250>::getNumOfViews(); }
//	
//	void addView(PointCloudPtr pCloud) { LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::addView(pCloud); }
//    PointCloudPtr getView(int i) { CloudjectModelBase<PointT,pcl::PFHRGBSignature250>::getView(i); }
//
//    void addViewDescriptor(DescriptorPtr pDescriptor) { LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::addViewDescriptor(pDescriptor); }
//    DescriptorPtr getViewDescriptor(int i) { return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::getViewDescriptor(i); }
//    
//	float euclideanDistance(PointT p1, PointT p2) { return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::euclideanDistance(p1,p2); }
//	float medianDistanceToCentroid(PointCloudPtr pCloud, PointT centroid)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::medianDistanceToCentroid(pCloud, centroid); }
//	
//	float averageNumOfPointsInModels() 
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::averageNumOfPointsInModels(); }
//	float averageMedianDistanceToCentroids() 
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::averageMedianDistanceToCentroids(); }
//
//	float match(LFCloudject c)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::match(c); }
//
//	int getPenalty()
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::getPenalty(); }
//
//
//	// Describe all the model views
//	void describe(float normalRadius, float fpfhRadius)
//	{
//		for (int i = 0; i < getNumOfViews(); i++)
//		{
//			DescriptorPtr pDescriptor (new Descriptor);
//			describeView(getView(i), normalRadius, fpfhRadius, *pDescriptor);
//			addViewDescriptor(pDescriptor);
//		}
//	}
//
//private:
//	float matchView(DescriptorPtr pDescriptor)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::matchView(pDescriptor); }
//
//	float battacharyyaDistanceSignatures(pcl::PFHRGBSignature250 s1, pcl::PFHRGBSignature250 s2)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::battacharyyaDistanceSignatures(s1, s2); }
//
//	float euclideanDistanceSignatures(pcl::PFHRGBSignature250 s1, pcl::PFHRGBSignature250 s2)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::euclideanDistanceSignatures(s1, s2); }
//
//	float euclideanDistanceSignatures(pcl::PFHRGBSignature250 s1, pcl::PFHRGBSignature250 s2, float thresh)
//	{ return LFCloudjectModelBase<PointT,pcl::PFHRGBSignature250>::euclideanDistanceSignatures(s1, s2, thresh); }
//
//
//	// Compute the description of a view, performing
//	// a prior downsampling to speed up the process
//	void describeView(PointCloudPtr pView, 
//					  float leafSize, float normalRadius, float pfhrgbRadius,
//					  Descriptor& descriptor)
//	{
//		PointCloudPtr pViewF (new PointCloud);
//
//		pcl::ApproximateVoxelGrid<PointT> avg;
//		avg.setInputCloud(pView);
//		avg.setLeafSize(leafSize, leafSize, leafSize);
//		avg.filter(*pViewF);
//
//		pViewF.swap(pView);
//
//		describeView(pView, normalRadius, pfhrgbRadius, descriptor);
//	}
//
//
//	// Compute the description of a view, actually
//	void describeView(PointCloudPtr pView, 
//					  float normalRadius, float pfhrgbRadius,
//					  Descriptor& descriptor)
//	{
//		//
//		// Normals preprocess
//		//
//
//		// Create the normal estimation class, and pass the input dataset to it
//		pcl::NormalEstimation<PointT,pcl::Normal> ne;
//		ne.setInputCloud (pView);
//
//		// Create an empty kdtree representation, and pass it to the normal estimation object.
//		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//		KdTreePtr tree (new KdTree());
//		ne.setSearchMethod (tree);
//
//		// Output datasets
//		pcl::PointCloud<pcl::Normal>::Ptr pNormals (new pcl::PointCloud<pcl::Normal>);
//
//		// Use all neighbors in a sphere of radius 3cm
//		ne.setRadiusSearch (normalRadius);
//
//		// Compute the features
//		ne.compute (*pNormals);	
//
//		//
//		// PFHRGB description extraction
//		//
//
//		pcl::PFHRGBEstimation<PointT,pcl::Normal,pcl::PFHRGBSignature250> pfhrgb;
//		pfhrgb.setInputCloud (pView);
//		pfhrgb.setInputNormals (pNormals);
//
//		// Create an empty kdtree representation, and pass it to the FPFH estimation object.
//		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//		tree = KdTreePtr(new KdTree());
//		pfhrgb.setSearchMethod (tree);
//
//		// Output datasets
//		// * initialize outside
//
//		// Use all neighbors in a sphere of radius 5cm
//		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//		pfhrgb.setRadiusSearch (pfhrgbRadius);
//
//		// Compute the features
//		pfhrgb.computeFeature (descriptor);
//	}
//};

// Explicit template instantation
template class CloudjectModelBase<pcl::PointXYZ,pcl::FPFHSignature33>;
//template class CloudjectModelBase<pcl::PointXYZ,pcl::PFHRGBSignature250>;
template class CloudjectModelBase<pcl::PointXYZRGB,pcl::FPFHSignature33>;
//template class CloudjectModelBase<pcl::PointXYZRGB,pcl::PFHRGBSignature250>;

template class LFCloudjectModelBase<pcl::PointXYZ,pcl::FPFHSignature33>;
//template class LFCloudjectModelBase<pcl::PointXYZ,pcl::PFHRGBSignature250>;
template class LFCloudjectModelBase<pcl::PointXYZRGB,pcl::FPFHSignature33>;
//template class LFCloudjectModelBase<pcl::PointXYZRGB,pcl::PFHRGBSignature250>;

template class LFCloudjectModel<pcl::PointXYZ,pcl::FPFHSignature33>;
//template class LFCloudjectModel<pcl::PointXYZ,pcl::PFHRGBSignature250>;
template class LFCloudjectModel<pcl::PointXYZRGB,pcl::FPFHSignature33>;
//template class LFCloudjectModel<pcl::PointXYZRGB,pcl::PFHRGBSignature250>;