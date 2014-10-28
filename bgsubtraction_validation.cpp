//
//  bgsubtraction_validation.cpp
//  remedi2
//
//  Created by Albert Clapés on 16/10/14.
//
//

#include "bgsubtraction_validation.h"

int loadBackgroundSubtractionValidationFile(string path, cv::Mat& combinations, vector<cv::Mat>& overlaps)
{
    combinations.release();
    overlaps.clear();
    
    cv::FileStorage fs;
    fs.open(path, cv::FileStorage::READ);
    
    if (!fs.isOpened()) // could not read error
        return -1;
    
    fs["combinations"] >> combinations;
    
    overlaps.resize(combinations.rows);
    for (int c = 0; c < combinations.rows; c++)
        fs["overlaps_" + to_str(c)] >> overlaps[c];
    
    fs.release();
    
    return 0;
}

void validateBackgroundSubtraction(ReMedi::Ptr pSys, vector<Sequence<ColorDepthFrame>::Ptr > sequences, vector< vector<double> > parameters, vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename, cv::Mat& combinations, vector<cv::Mat>& overlaps)
{
    pSys->initialize();
    overlaps.clear();
    
    Sequence<ColorDepthFrame>::Ptr pBgSeq = pSys->getBackgroundSequence();
    
    // Some consistency checks
    
    int numOfViews = pBgSeq->getNumOfViews();
    int numOfForegroundMasks = fgMasksSequences[0]->getMinNumOfFrames();
    
    for (int s = 0; s < sequences.size(); s++)
        assert (numOfViews == sequences[s]->getNumOfViews());
    
    for (int s = 0; s < fgMasksSequences.size(); s++)
    {
        assert (numOfViews == fgMasksSequences[s]->getNumOfViews());
        for (int v = 0; v < numOfViews; v++)
            assert (numOfForegroundMasks == fgMasksSequences[s]->getMinNumOfFrames());
    }
    
    // Backup file
    
    boost::filesystem::path filePath (path + filename);
    boost::filesystem::path fileBakPath (path + filePath.stem().string() + ".bak" + filePath.extension().string());
    if (boost::filesystem::exists(filePath))
        boost::filesystem::copy_file(filePath, fileBakPath, boost::filesystem::copy_option::overwrite_if_exists);
    
    // Create subtractor
    
    BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs = pSys->getBackgroundSubtractor();
    pBs->setFramesResolution(X_RESOLUTION, Y_RESOLUTION);
    pBs->setBackgroundSequence(pBgSeq, BS_NUM_OF_SAMPLES);
    
    // Prepare combinations of parameters to test
    
    expandParameters<double>(parameters, combinations);
    
    // New combinations to compute? Check which are they
    
    cv::Mat combinationsTmp;
    vector<cv::Mat> overlapsTmp;
    string filepath = path + filename;
    loadBackgroundSubtractionValidationFile(filepath, combinationsTmp, overlapsTmp);
    
    cv::FileStorage fs;
    int offset = 0;
    cv::Mat pending;
    
    if (combinationsTmp.empty())
    {
        fs.open(path + filename, cv::FileStorage::WRITE);
        fs << "combinations" << combinations;
        fs.release();
        
        pending = combinations;
    }
    else
    {
        cv::Mat matched;
        cvx::match(combinationsTmp, combinations, matched, pending);
        
        // Create a file to store the previous combinations' results and the new (if needed)
        if (pending.empty()) combinations = combinationsTmp;
        else cv::vconcat(combinationsTmp, pending, combinations);
        
        fs.open(path + filename, cv::FileStorage::WRITE);
        fs << "combinations" << combinations;
        for (int c = 0; c < combinationsTmp.rows; c++)
        {
            overlaps.push_back(overlapsTmp[c]);
            fs << "overlaps_" + to_str(c) << overlapsTmp[c];
        }
        fs.release();
        
        offset = combinationsTmp.rows;
    }
    
    // Compute the new
    
    cv::Mat prevc;
    if (!pending.empty()) prevc.create(1, pending.cols, pending.type());
    
    for (int c = 0; c < pending.rows; c++)
    {
        string parametersStr = to_string_with_precision<double>(pending.row(c), "_", 4);
        cout << c << "/" << pending.rows << " : " << parametersStr << " ";
        
        pBs->setModality(pending.at<double>(c,0));
        pBs->setNumOfMixtureComponents(pending.at<double>(c,1));
        pBs->setLearningRate(pending.at<double>(c,2));
        pBs->setBackgroundRatio(pending.at<double>(c,3));
        pBs->setVarThresholdGen(pending.at<double>(c,4));
        pBs->setOpeningSize(pending.at<double>(c,5));
        
        boost::timer t;
        
        int idx = cvx::diffIdx(pending.row(c), prevc).x;
        if (idx < 5)
            pBs->model();
        prevc = pending.row(c);
        
        vector<cv::Mat> seqsOverlaps;
        
        for (int s = 0; s < fgMasksSequences.size(); s++)
        {
            Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
            Sequence<ColorDepthFrame>::Ptr pTestSeq = sequences[s];
            
            string seqPath = path + to_str(s+1) + "/";
            
            cv::Mat seqOverlaps (pFgGtSeq->getNumOfViews(), pFgGtSeq->getMinNumOfFrames(), cv::DataType<float>::type);
            
            int f = 0;
            pFgGtSeq->restart();
            while (pFgGtSeq->hasNextFrames())
            {
                vector<Frame::Ptr> fgGtMasksFrames = pFgGtSeq->nextFrames();
                
                // search for correspondence using filename
                vector<string> filenames = pFgGtSeq->getFramesFilenames();
                vector<ColorDepthFrame::Ptr> testFrames = pTestSeq->getFrames(filenames);
                pSys->getRegisterer()->setInputFrames(testFrames);
                pSys->getRegisterer()->registrate(testFrames);
                
                // focus only on tabletop's region for the BS
                vector<cv::Mat> tabletopMasks;
                pSys->getTableModeler()->getTabletopMask(testFrames, tabletopMasks);
                
                // test
                vector<cv::Mat> foregroundMasks;
                pBs->setInputFrames(testFrames);
                pBs->subtract(foregroundMasks);
                
                // gather the test results
                for (int v = 0; v < pBgSeq->getNumOfViews(); v++)
                {
                    // Masks were in disk as RGB images (preprocessing)
                    cv::Mat fgGtGrayMask, fgGtBinMask;
                    cv::cvtColor(fgGtMasksFrames[v]->get(), fgGtGrayMask, CV_BGR2GRAY);
                    cv::threshold(fgGtGrayMask, fgGtBinMask, 0, 255, cv::THRESH_BINARY);
                    
                    cv::Mat maskedPrediction;
                    maskedPrediction = foregroundMasks[v] & tabletopMasks[v];
                    
                    // Quantitative result (overlap)
                    seqOverlaps.at<float>(v,f) = cvx::overlap(maskedPrediction, fgGtBinMask);
                    
                    // Qualitative result (mask image)
                    //string maskPath = seqPath + "ForegroundMasks" + to_str(v+1) + "/";
                    //cv::imwrite(maskPath + filenames[v] + "-" + parametersStr + ".png", maskedPrediction);
                }
                
                f++;
            }
            
            seqsOverlaps.push_back(seqOverlaps);
        }
        
        cv::Mat combinationOverlaps;
        cvx::hconcat(seqsOverlaps, combinationOverlaps);
        
        fs.open(path + filename, cv::FileStorage::APPEND);
        fs << "overlaps_" + to_str(offset + c) << combinationOverlaps;
        fs.release();
        
        overlaps.push_back(combinationOverlaps);
        
        cout << t.elapsed() << endl;
    }
    
    if (boost::filesystem::exists(fileBakPath))
        boost::filesystem::remove(fileBakPath);
}

void summarizeBackgroundSubtractionValidation(vector<Sequence<Frame>::Ptr> foregroundMasksSequences, std::vector<int> subjectsIds, cv::Mat combinations, vector<cv::Mat> overlaps, std::vector<cv::Mat>& summaries)
{
    assert (combinations.rows == overlaps.size());
    
    summaries.clear();
    
    for (int v = 0; v < NUM_OF_VIEWS; v++)
    {
        // A matrix per view. Each matrix has as many rows as combinations, and
        // as many columns as #{sequences} x #{foreground masks in each sequence}.
        // In other words is a concenation of the v-th rows of "overlaps" matrices
        cv::Mat overlapsView (overlaps.size(), overlaps[0].cols, overlaps[0].type());
        for (int i = 0; i < overlaps.size(); i++)
        {
            assert (overlaps[i].cols == overlaps[0].cols);
            
            overlaps[i].row(v).copyTo(overlapsView.row(i));
        }
        
        // Reduce the columns of the same sequence, averaging them in one
        // This results to as many colums as sequences. These are the mean overlap
        // in each sequence;
        
        cv::Mat overlapsViewSeqsSmry (overlaps.size(), foregroundMasksSequences.size(), overlapsView.type());
        
        int seqstart = 0;
        for (int i = 0; i < foregroundMasksSequences.size(); i++)
        {
            cv::Mat roi (overlapsView, cv::Rect(seqstart, 0, foregroundMasksSequences[i]->getMinNumOfFrames(), overlapsView.rows));
            cv::reduce(roi, overlapsViewSeqsSmry.col(i), 1, CV_REDUCE_AVG);
            
            seqstart += foregroundMasksSequences[i]->getMinNumOfFrames();
        }
        
        summaries.push_back(overlapsViewSeqsSmry);
    }
}