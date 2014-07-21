#include <iostream>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/timer.hpp>

#include "Remedi.h"
#include "KinectReader.h"
#include "Sequence.h"
#include "ColorDepthFrame.h"

#include "statistics.h"

using namespace std;
using namespace boost::assign;

// KinectReader's constants
#define SEQUENCES_RPATH                 "../../../Data/Sequences/"
#define KINECT_SUBDIR                   "Kinects/"

#define KINECT_NUM_OF_VIEWS             2

#define COLOR_DIRNAME_1                 "Color1/"
#define COLOR_DIRNAME_2                 "Color2/"
#define DEPTH_DIRNAME_1                 "Depth1/"
#define DEPTH_DIRNAME_2                 "Depth2/"

#define READER_DELAY_1                  2
#define READER_DELAY_2                  0

// InteractiveRegisterer's constants
#define IR_VIS_WND_HEIGHT               480
#define IR_VIS_WND_WIDTH                640
#define IR_VIS_VP                       1
#define IR_VIS_HP                       2
#define IR_VIS_DIST                     -2 // -2 meters
#define IR_VIS_MARKER_RADIUS            0.015
#define IR_DEFAULT_FRAME                2
#define IR_NUM_OF_POINTS                -1 // 5

// BackgroundSubtractor's constants
#define BS_NUM_OF_SAMPLES               400
#define BS_MODALITY                     0
#define BS_K                            5
#define BS_LRATE                        -1
#define BS_BGRATIO                      0.999
#define BS_OPENING                      2

// TableModeler's constants
#define TM_LEAF_SIZE                    0.02
#define TM_NORMAL_RADIUS                0.05
#define TM_SAC_ITERS                    200
#define TM_SAC_DIST_THRESH              0.03
#define TM_TT_Y_OFFSET                  0.4 // tabletop y-dim offset
#define TM_INTERACTIVE_BORDER_DIST      0.7
#define TM_CONF_LEVEL                   99

// Monitorizer's constants
#define MO_LEAF_SIZE                    0.005
#define MO_CLUSTERS_DIST_FACTOR         3

// THIS FUNCTION has to be adapted to the data structures of the database
void loadKinectFramesPaths(string parent, string subdir, int n, vector<string> colorDirs, vector<string> depthDirs, vector< vector< pair<string,string> > >& paths)
{
    // Creates such structure:
    // nviews * nframes * (color,depth)-pair
    
 	paths.clear(); // a list of paths per view
    
    const char* path = parent.c_str();
	if( boost::filesystem::exists( path ) )
	{
		boost::filesystem::directory_iterator end;
		boost::filesystem::directory_iterator iter(path);
		for( ; iter != end ; ++iter )
		{
			if ( boost::filesystem::is_directory( *iter ) )
            {
                string pathTmp = iter->path().string() + "/" + subdir;
                
                vector< pair<string,string> > viewPaths (n);
                
                for (int v = 0; v < n; v++)
                {
                    string colorPathTmp = pathTmp + colorDirs[v];
                    string depthPathTmp = pathTmp + depthDirs[v];
                    viewPaths[v] = ( pair<string,string>(colorPathTmp, depthPathTmp) );
                }
                
                paths.push_back(viewPaths);
            }
		}
	}
}

int main()
{
    vector<string> colorDirs, depthDirs;
    colorDirs += COLOR_DIRNAME_1, COLOR_DIRNAME_2;
    depthDirs += DEPTH_DIRNAME_1, DEPTH_DIRNAME_2;
    
    vector< vector< pair<string,string> > > sequencesPaths;
    loadKinectFramesPaths(SEQUENCES_RPATH, KINECT_SUBDIR, KINECT_NUM_OF_VIEWS, colorDirs, depthDirs, sequencesPaths);
    
    // *----------------------------*
    // | Read Kinect data sequences |
    // *----------------------------*
    KinectReader reader;
    
    // background sequence
    Sequence<ColorDepthFrame>::Ptr pBgSeq (new Sequence<ColorDepthFrame>(KINECT_NUM_OF_VIEWS));
    reader.read(sequencesPaths[0], *pBgSeq);
    
    // and the rest of them
    vector<Sequence<ColorDepthFrame>::Ptr> pSequences;
    for (int s = 1; s < sequencesPaths.size(); s++)
    {
        Sequence<ColorDepthFrame>::Ptr pSeq (new Sequence<ColorDepthFrame>(KINECT_NUM_OF_VIEWS));
        reader.read(sequencesPaths[s], *pSeq);
        pSequences.push_back(pSeq);
    }
    
    // *-----------------------------------*
    // | Create and parametrize the system |
    // *-----------------------------------*
    ReMedi sys;
    sys.setBackgroundSequence(pBgSeq);
    sys.setInputSequences(pSequences);
    
    sys.setRegistererParameters(IR_NUM_OF_POINTS,IR_VIS_WND_HEIGHT, IR_VIS_WND_WIDTH, IR_VIS_VP, IR_VIS_HP, IR_VIS_DIST, IR_VIS_MARKER_RADIUS, IR_DEFAULT_FRAME);
    sys.setTableModelerParameters(TM_LEAF_SIZE, TM_NORMAL_RADIUS, TM_SAC_ITERS, TM_SAC_DIST_THRESH, TM_TT_Y_OFFSET, TM_INTERACTIVE_BORDER_DIST, TM_CONF_LEVEL);
    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_K, BS_LRATE, BS_BGRATIO, BS_OPENING);
    sys.setMonitorizerParameters(MO_LEAF_SIZE, MO_CLUSTERS_DIST_FACTOR);
    
    // *------------------------------------*
    // | Validation of BackgroundSubtractor |
    // *------------------------------------*
    
    // the set of parameters to validate and their combinations
    vector<vector<double> > bsparams, expbsparams;
    
    // specify the values for each
    vector<double> modalities;
    modalities += ReMedi::COLOR, ReMedi::DEPTH;//, ReMedi.COLORDEPTH, ReMedi.COLORDEPTHNORMALS; // use enum instead, e.g. modalities += HS, D, HSD, HSDN;
    vector<double> components;
    components += 2, 5, 10;
    vector<double> learningRates;
    learningRates += 0, 0.01, 0.05;
    vector<double> backgroundRatios;
    backgroundRatios += 0.5, 0.9, 0.999;
    vector<double> morphologyLevels;
    morphologyLevels += 0, 1, 2;

    bsparams += modalities, components, learningRates, backgroundRatios, morphologyLevels;
    expandParameters<double>(bsparams, expbsparams);
    
    sys.validateBS(expbsparams);
    
//    // ******************************************
//    sys.run(); // normal run
//    // ******************************************
    
	return 0;
}