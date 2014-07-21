//
//  KinectReader.cpp
//  remedi2
//
//  Created by Albert Clapés on 13/07/14.
//
//

#include "KinectReader.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

KinectReader::KinectReader() {}

KinectReader::KinectReader(const KinectReader& rhs)
{
    *this = rhs;
}

KinectReader& KinectReader::operator=(const KinectReader& rhs)
{
    if (this != &rhs)
    {
        m_bPreallocation = rhs.m_bPreallocation;
    }
    
    return *this;
}

void KinectReader::setPreallocation(bool prealloc)
{
    m_bPreallocation = prealloc;
}

void KinectReader::read(vector< pair<string,string> > paths, Sequence<ColorFrame>& seq)
{
    for (int v = 0; v < paths.size(); v++)
    {
        vector<string> colorDirs;
        loadFilePaths(paths[v].first, "png", colorDirs);
        
        for (int f = 0; f < colorDirs.size(); f++)
        {
            seq.addFrameFilePath(colorDirs[f], v);
        }
    }
    
    if (m_bPreallocation) seq.allocate();
}

void KinectReader::read(vector< pair<string,string> > paths, Sequence<DepthFrame>& seq)
{
    for (int v = 0; v < paths.size(); v++)
    {
        vector<string> depthDirs;
        loadFilePaths(paths[v].first, "png", depthDirs);
        
        for (int f = 0; f < depthDirs.size(); f++)
        {
            seq.addFrameFilePath(depthDirs[f], v);
        }
    }
    
    if (m_bPreallocation) seq.allocate();
}

void KinectReader::read(vector< pair<string,string> > paths, Sequence<ColorDepthFrame>& seq)
{
    for (int v = 0; v < paths.size(); v++)
    {
        vector<string> colorDirs, depthDirs;
        loadFilePaths(paths[v].first, "png", colorDirs);
        loadFilePaths(paths[v].second, "png", depthDirs);
        
        assert ( colorDirs.size() == depthDirs.size() );
        
        for (int f = 0; f < colorDirs.size(); f++)
        {
            seq.addFramesFilePath(colorDirs[f], depthDirs[f], v);
        }
    }
    
    if (m_bPreallocation) seq.allocate();
}

void KinectReader::loadFilePaths(string dir, const char* filetype, vector<string>& filePaths)
{
 	filePaths.clear();
    
    string extension = "." + string(filetype);
    const char* path = dir.c_str();
	if( boost::filesystem::exists( path ) )
	{
		boost::filesystem::directory_iterator end;
		boost::filesystem::directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( !boost::filesystem::is_directory( *iter ) && (iter->path().extension().string().compare(extension) == 0) )
            {
                string aux = iter->path().string();
				filePaths.push_back(iter->path().string());
            }
		}
	}
}