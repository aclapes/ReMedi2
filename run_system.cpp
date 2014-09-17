#include <iostream>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/timer.hpp>

#include "Remedi.h"
#include "constants.h"

#include "KinectReader.h"
#include "Sequence.h"
#include "ColorDepthFrame.h"
#include "DetectionOutput.h"

#include "statistics.h"
#include "cvxtended.h"

using namespace std;
using namespace boost::assign;

int run()
{    
    vector<string> colorDirNames, depthDirNames;
    colorDirNames += COLOR_DIRNAME_1, COLOR_DIRNAME_2;
    depthDirNames += DEPTH_DIRNAME_1, DEPTH_DIRNAME_2;
    
    vector<string> sequencesNames;
    ReMedi::loadSequences(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), sequencesNames);
    
    vector< vector<string> > colorDirPaths, depthDirPaths;
    ReMedi::loadDirPaths(string(PARENT_PATH) + string(SEQUENCES_SUBDIR),
                 sequencesNames, KINECT_SUBSUBDIR, colorDirNames, colorDirPaths);
    ReMedi::loadDirPaths(string(PARENT_PATH) + string(SEQUENCES_SUBDIR),
                 sequencesNames, KINECT_SUBSUBDIR, depthDirNames, depthDirPaths);
    
    assert(colorDirPaths.size() == depthDirPaths.size());
    
    // *----------------------------*
    // | Read Kinect data sequences |
    // *----------------------------*
    KinectReader reader;
    
    // background sequence
    Sequence<ColorDepthFrame>::Ptr pBgSeq (new Sequence<ColorDepthFrame>(NUM_OF_VIEWS));
    reader.setPreallocation();
    reader.read(colorDirPaths[0], depthDirPaths[0], *pBgSeq);
    reader.setPreallocation(false);

    // and the rest of them (taking account the delays among views)
    vector<vector<int> > delays;
    ReMedi::loadDelaysFile(string(PARENT_PATH) + string(SEQUENCES_SUBDIR), string(DELAYS_FILENAME), delays);
    
    vector<Sequence<ColorDepthFrame>::Ptr> sequences;
    for (int s = 1; s < colorDirPaths.size(); s++) // skip BG sequence
    {
        Sequence<ColorDepthFrame>::Ptr pSeq (new Sequence<ColorDepthFrame>(NUM_OF_VIEWS));
        reader.read(colorDirPaths[s], depthDirPaths[s], *pSeq);
        pSeq->setName(sequencesNames[s]);
        pSeq->setDelays(delays[s]);
        
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
    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_K, BS_LRATE, BS_BGRATIO, BS_VARGEN);
//    sys.setSubtractorParameters(BS_NUM_OF_SAMPLES, BS_MODALITY, BS_MAX_FEATURES, BS_LRATE, BS_QUANTIZATION_LEVELS, BS_DECISIONVAR);

    sys.setObjectDetectorParameters(OD_MORPHOLOGY, OD_LEAF_SIZE, OD_CLUSTERS_DIST_FACTOR, OD_MINIMUM_CLUSTER_SIZE);
    
    sys.run();

	return 0;
}

//int main()
//{
//    return run();
//}
