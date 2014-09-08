//
//  constants.h
//  remedi2
//
//  Created by Albert Clapés on 30/08/14.
//
//

#ifndef remedi2_constants_h
#define remedi2_constants_h

// KinectReader's constants
#define PARENT_PATH                     "../../../Data/"
#define SEQUENCES_SUBDIR                "Sequences/"
#define KINECT_SUBSUBDIR                "Kinects/"
#define OBJECTLABELS_SUBDIR             "ObjectLabels/"

#define KINECT_NUM_OF_VIEWS             2
#define DELAYS_FILENAME                 "delays.txt"

#define COLOR_DIRNAME_1                 "Color1/"
#define COLOR_DIRNAME_2                 "Color2/"
#define DEPTH_DIRNAME_1                 "Depth1/"
#define DEPTH_DIRNAME_2                 "Depth2/"

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
#define BS_MODALITY                     3
#define BS_LRATE                        -1
#define BS_OPENING                      2

#define BS_K                            30
#define BS_BGRATIO                      0.99999
#define BS_VARGEN                       25
//#define BS_MAX_FEATURES                 4
//#define BS_QUANTIZATION_LEVELS          2
//#define BS_DECISIONVAR                  0.99

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
#define MO_CLUSTERS_DIST_FACTOR         5
#define MO_MINIMUM_CLUSTER_SIZE         50

#endif
