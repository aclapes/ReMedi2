//
//  ObjectModel.hpp
//  remedi2
//
//  Created by Albert Clapés on 15/09/14.
//
//

#ifndef __remedi2__ObjectModel__
#define __remedi2__ObjectModel__

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

template<typename PointT>
class ObjectModelBase
{
public:
    ObjectModelBase() {}
    ObjectModelBase(int oid, string name) : m_OID(oid), m_Name(name)
    {}
    ObjectModelBase(const ObjectModelBase& rhs)
    { *this = rhs; }
    
    ObjectModelBase& operator=(const ObjectModelBase& rhs)
    {
        if (this != &rhs)
        {
            m_OID = rhs.m_OID;
            m_Name = rhs.m_Name;
        }
        
        return *this;
    }
    
    int getID() { return m_OID; }
    string getName() { return m_Name; }
    
    void setID(int oid) { m_OID = oid; }
    void setName(string name) { m_Name = name; };
    
private:
    int m_OID;
    string m_Name;
};

template<typename PointT>
class ObjectModel : public ObjectModelBase<PointT>
{};

template<>
class ObjectModel<pcl::PointXYZ> : public ObjectModelBase<pcl::PointXYZ>
{
public:
    ObjectModel()
    : ObjectModelBase<pcl::PointXYZ>() {}
    ObjectModel(int oid, string name)
    : ObjectModelBase<pcl::PointXYZ>(oid, name)
    {}
    ObjectModel(int oid, string name, vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views)
    : ObjectModelBase<pcl::PointXYZ>(oid, name), m_Views(views)
    {}
    ObjectModel(const ObjectModel& rhs)
    { *this = rhs; }
    
    ObjectModel& operator=(const ObjectModel& rhs)
    {
        if (this != &rhs)
        {
            ObjectModelBase<pcl::PointXYZ>::operator=(rhs);
            m_Views = rhs.m_Views;
        }
        return *this;
    }
    
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getViews() { return m_Views; }
    
    void setViews(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> views) { m_Views = views; }
    
    typedef boost::shared_ptr<ObjectModel<pcl::PointXYZ> > Ptr;
    
private:
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_Views;
};

template<>
class ObjectModel<pcl::PointXYZRGB> : public ObjectModelBase<pcl::PointXYZRGB>
{
public:
    ObjectModel()
      : ObjectModelBase<pcl::PointXYZRGB>() {}
    ObjectModel(int oid, string name)
      : ObjectModelBase<pcl::PointXYZRGB>(oid, name)
    {}
    ObjectModel(int oid, string name, vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> views)
    : ObjectModelBase<pcl::PointXYZRGB>(oid, name), m_Views(views)
    {}
    ObjectModel(const ObjectModel& rhs)
    { *this = rhs; }
    
    ObjectModel& operator=(const ObjectModel& rhs)
    {
        if (this != &rhs)
        {
            ObjectModelBase<pcl::PointXYZRGB>::operator=(rhs);
            m_Views = rhs.m_Views;
        }
        return *this;
    }
    
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getViews() { return m_Views; }
    
    void setViews(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> views) { m_Views = views; }
    
    typedef boost::shared_ptr<ObjectModel<pcl::PointXYZRGB> > Ptr;
    
private:
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_Views;
};

#endif /* defined(__remedi2__ObjectModel__) */
