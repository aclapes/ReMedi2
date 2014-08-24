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

#define DELAY_1                         2
#define DELAY_2                         0

#define DEFAULT_FRAME                   2

#define Y_RESOLUTION                    480
#define X_RESOLUTION                    640

// InteractiveRegisterer's constants
#define IR_VIS_WND_HEIGHT               480
#define IR_VIS_WND_WIDTH                640
#define IR_VIS_VP                       1
#define IR_VIS_HP                       2
#define IR_VIS_DIST                     -2 // -2 meters
#define IR_VIS_MARKER_RADIUS            0.015
#define IR_NUM_OF_POINTS                -1

// BackgroundSubtractor's constants
#define FOREGROUND_MASKS_DIRNAME_1      "ForegroundMasks1/"
#define FOREGROUND_MASKS_DIRNAME_2      "ForegroundMasks2/"

#define BS_COMPUTE_MODELS               FALSE

#define BS_NUM_OF_SAMPLES               400
#define BS_MODALITY                     0
#define BS_LRATE                        -1
#define BS_OPENING                      2

#ifdef BS_USE_MOG2
#define BS_K                            10
#define BS_BGRATIO                      0.9999
#define BS_VARGEN                       9
#else
#define BS_MAX_FEATURES                 4
#define BS_QUANTIZATION_LEVELS          2
#define BS_DECISIONVAR                  0.99
#endif // BS_USE_MOG2

// TableModeler's constants
#define TM_LEAF_SIZE                    0.01
#define TM_NORMAL_RADIUS                0.05
#define TM_SAC_ITERS                    200
#define TM_SAC_DIST_THRESH              0.03
#define TM_TT_Y_OFFSET                  0.4 // tabletop y-dim offset
#define TM_INTERACTIVE_BORDER_DIST      0.7
#define TM_CONF_LEVEL                   99

// Monitorizer's constants
#define MO_LEAF_SIZE                    0.005
#define MO_CLUSTERS_DIST_FACTOR         3

void loadDirPaths(string parent, string subdir, int n, vector<string> DirNames, vector< vector<string> >& DirPaths)
{
 	DirPaths.clear(); // a list of paths per view
    
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
                
                vector<string> viewPaths (n);
                for (int v = 0; v < n; v++)
                    viewPaths[v] = pathTmp + DirNames[v];
                
                DirPaths.push_back(viewPaths);
            }
		}
	}
}

int main()
{
    vector<string> colorDirNames, depthDirNames;
    colorDirNames += COLOR_DIRNAME_1, COLOR_DIRNAME_2;
    depthDirNames += DEPTH_DIRNAME_1, DEPTH_DIRNAME_2;
    
    vector< vector<string> > colorDirPaths, depthDirPaths;
    loadDirPaths(SEQUENCES_RPATH, KINECT_SUBDIR, KINECT_NUM_OF_VIEWS, colorDirNames, colorDirPaths);
    loadDirPaths(SEQUENCES_RPATH, KINECT_SUBDIR, KINECT_NUM_OF_VIEWS, depthDirNames, depthDirPaths);
    
    assert(colorDirPaths.size() == depthDirPaths.size());
    
    // *----------------------------*
    // | Read Kinect data sequences |
    // *----------------------------*
    KinectReader reader;
    
    // background sequence
    Sequence<ColorDepthFrame>::Ptr pBgSeq (new Sequence<ColorDepthFrame>(KINECT_NUM_OF_VIEWS));
    reader.setPreallocation();
    reader.read(colorDirPaths[0], depthDirPaths[0], *pBgSeq);
    reader.setPreallocation(false);

    // and the rest of them
    vector<int> delays;
    delays += DELAY_1, DELAY_2;
    
    vector<Sequence<ColorDepthFrame>::Ptr> sequences;
    for (int s = 1; s < colorDirPaths.size(); s++) // skip BG sequence
    {
        Sequence<ColorDepthFrame>::Ptr pSeq (new Sequence<ColorDepthFrame>(KINECT_NUM_OF_VIEWS));
        reader.read(colorDirPaths[s], depthDirPaths[s], *pSeq);
        pSeq->setDelays(delays);
        
        sequences.push_back(pSeq);
    }
    
    // *-----------------------------------*
    // | Create and parametrize the system |
    // *-----------------------------------*
    ReMedi sys;
    sys.setFramesResolution(X_RESOLUTION, Y_RESOLUTION);
    sys.setBackgroundSequence(pBgSeq);
    sys.setInputSequences(sequences);
    sys.setDefaultFrame(DEFAULT_FRAME);
    
    sys.setRegistererParameters(IR_NUM_OF_POINTS, IR_VIS_WND_HEIGHT, IR_VIS_WND_WIDTH, IR_VIS_VP, IR_VIS_HP, IR_VIS_DIST, IR_VIS_MARKER_RADIUS);
    sys.setTableModelerParameters(TM_LEAF_SIZE, TM_NORMAL_RADIUS, TM_SAC_ITERS, TM_SAC_DIST_THRESH, TM_TT_Y_OFFSET, TM_INTERACTIVE_BORDER_DIST, TM_CONF_LEVEL);
#ifdef BS_USE_MOG2
    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_K, BS_LRATE, BS_BGRATIO, BS_VARGEN, BS_OPENING);
#else
    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_MAX_FEATURES, BS_LRATE, BS_QUANTIZATION_LEVELS, BS_DECISIONVAR, BS_OPENING);
#endif // BS_USE_MOG2
    sys.setMonitorizerParameters(MO_LEAF_SIZE, MO_CLUSTERS_DIST_FACTOR);
    
    // *------------------------------------*
    // | Validation of BackgroundSubtractor |
    // *------------------------------------*
    
    // Build the set of parameters to validate the subtraction
    
    vector<vector<double> > bsParameters;
    
    // Specify the values of each
    
    vector<double> modalities;
    vector<double> learningRates;
    learningRates += 0;
    vector<double> openingSizes;
    openingSizes += 0;
    
    vector<string> varNames;
    varNames += "modality";
    
#ifdef BS_USE_MOG2
    modalities += ReMedi::COLOR, ReMedi::DEPTH, ReMedi::COLOR_WITH_SHADOWS, ReMedi::COLORDEPTH;
    vector<double> components;
    components += 2, 3, 4, 5, 6, 8, 10, 12, 15, 18, 20, 25, 30;
    vector<double> backgroundRatios;
    backgroundRatios += 0.9, 0.99, 0.99999;
    vector<double> varGenThresholds;
    varGenThresholds += 0.75, 1.5, 3, 6, 9, 12, 15, 18, 25;

    bsParameters += modalities, components, learningRates, backgroundRatios, varGenThresholds, openingSizes;
    
    varNames += "#mixture_components", "learning_rate", "background_ratio", "var_threshold_gen", "opening_size";
    writeParametersToFile("bs-mog2_results/parameters.txt", varNames, bsParameters, true);
#else
    modalities += /*ReMedi::COLOR, ReMedi::DEPTH, ReMedi::COLOR_WITH_SHADOWS,*/ ReMedi::COLORDEPTH;
    vector<double> maxFeatures;
    maxFeatures += 2, 3, 4, 5, 6, 7, 8, 16, 32;
    vector<double> quantizationLevels;
    quantizationLevels += 1, 2, 3, 4, 5, 6, 7, 8, 16, 32;
    vector<double> decisionThresholds;
    decisionThresholds += 0.8, 0.9, 0.999, 0.99999;
    
    bsParameters += modalities, maxFeatures, learningRates, quantizationLevels, decisionThresholds, openingSizes;
    
    varNames += "#max_features", "learning_rate", "#quantization_levels", "var_decision_threshold", "opening_size";
    writeParametersToFile("bs-gmg_results/parameters.txt", varNames, bsParameters, true);
#endif // BS_USE_MOG2
    
    // Get groundtruth foreground masks
    
    vector<string> foregroundMasksDirNames;
    foregroundMasksDirNames += FOREGROUND_MASKS_DIRNAME_1, FOREGROUND_MASKS_DIRNAME_2;
    
    vector< vector<string> > foregroundMasksDirPaths;
    loadDirPaths(SEQUENCES_RPATH, KINECT_SUBDIR, KINECT_NUM_OF_VIEWS, foregroundMasksDirNames, foregroundMasksDirPaths);
    
    vector<Sequence<Frame>::Ptr> foregroundMasksSequences;
    for (int s = 1; s < foregroundMasksDirPaths.size(); s++) // skip BG sequence
    {
        Sequence<Frame>::Ptr pFgSeq (new Sequence<Frame>(KINECT_NUM_OF_VIEWS));
        reader.read(foregroundMasksDirPaths[s], *pFgSeq);
        
        foregroundMasksSequences.push_back(pFgSeq);
    }
    
    // Validate!
    
//    sys.validateBS(bsParameters, foregroundMasksSequences);
    
//    sys.showBsParametersPerformance(foregroundMasksSequences, "validation.yml");
    vector<vector<double> > bsCombinations;
    sys.getBsParametersPerformance("validation.yml", bsCombinations);

    sys.validateMonitorizerClustering();
    

    
//    // ******************************************
//    sys.run(); // normal run
//    // ******************************************
    
	return 0;
}