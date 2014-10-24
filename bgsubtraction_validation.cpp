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
        fs["overlaps_" + to_string(c)] >> overlaps[c];
    
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
            fs << "overlaps_" + to_string(c) << overlapsTmp[c];
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
            
            string seqPath = path + to_string(s+1) + "/";
            
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
                    //string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
                    //cv::imwrite(maskPath + filenames[v] + "-" + parametersStr + ".png", maskedPrediction);
                }
                
                f++;
            }
            
            seqsOverlaps.push_back(seqOverlaps);
        }
        
        cv::Mat combinationOverlaps;
        cvx::hconcat(seqsOverlaps, combinationOverlaps);
        
        fs.open(path + filename, cv::FileStorage::APPEND);
        fs << "overlaps_" + to_string(offset + c) << combinationOverlaps;
        fs.release();
        
        overlaps.push_back(combinationOverlaps);
        
        cout << t.elapsed() << endl;
    }
    
    if (boost::filesystem::exists(fileBakPath))
        boost::filesystem::remove(fileBakPath);
}

void summarizeBackgroundSubtractionValidation(cv::Mat combinations, vector<cv::Mat> overlaps, cv::Mat& means, cv::Mat& stddevs)
{
    assert (combinations.rows == overlaps.size());
    
    means.create(overlaps.size(), overlaps[0].rows, cv::DataType<float>::type);
    stddevs.create(overlaps.size(), overlaps[0].rows, cv::DataType<float>::type);
    for (int c = 0; c < overlaps.size(); c++)
    {
        assert (overlaps[0].rows == overlaps[c].rows);
        for (int v = 0; v < overlaps[c].rows; v++)
        {
            cv::Scalar m, s;
            cv::meanStdDev(overlaps[c].row(v), m, s);
            means.at<float>(c,v) = m.val[0];
            stddevs.at<float>(c,v) = s.val[0];
        }
    }
}

void showBsParametersPerformance(vector<Sequence<Frame>::Ptr> fgMasksSequences, string path, string filename)
{
    //    cv::Mat combinations, overlaps;
    //
    //    cv::FileStorage fs;
    //    fs.open(path + filename, cv::FileStorage::READ);
    //    fs["combinations"] >> combinations;
    //    fs["overlaps"] >> overlaps;
    //    fs.release();
    //
    //    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
    //    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
    //    {
    //        cv::Rect roi (v * numOfFrames, 0, numOfFrames, combinations.rows);
    //        cv::Mat overlapsView (overlaps, roi);
    //
    //        cv::Mat overlapAvgView;
    //        cv::reduce(overlapsView, overlapAvgView, 1, CV_REDUCE_AVG);
    //
    //        cv::Mat I;
    //        cv::sortIdx(overlapAvgView, I, cv::SORT_EVERY_COLUMN | cv::SORT_DESCENDING);
    //
    //        int best = 10;
    //        for (int i = 0; i < best; i++)
    //        {
    //            int idx = I.at<int>(0,i);
    //            cout << i << "/" << best << combinations.row(idx) << "\t" << overlapAvgView.row(idx) << endl;
    //
    //            for (int s = 3; s < 4 /*fgMasksSequences.size()*/; s++)
    //            {
    //                string seqPath = path + to_string(s+1) + "/";
    //                string maskPath = seqPath + "ForegroundMasks" + to_string(v+1) + "/";
    //
    //                Sequence<Frame>::Ptr pFgGtSeq = fgMasksSequences[s];
    //                pFgGtSeq->restart();
    //                //while (pFgGtSeq->hasNextFrames())
    //                //{
    //                vector<Frame::Ptr> frames = pFgGtSeq->nextFrames();
    //                vector<string> filenames = pFgGtSeq->getFramesFilenames();
    //
    //                cout << v << " " << i << " " << s << " " << filenames[v] << endl;
    //
    //                string filename = filenames[v] + "-"; // then concantenated with params combination
    //                filename += to_string_with_precision(combinations.at<double>(idx,0), 4);
    //                for (int p = 1; p < combinations.cols; p++)
    //                    filename += "_" + to_string_with_precision(combinations.at<double>(idx,p), 4);
    //
    //                cv::Mat maskedPrediction;
    //                maskedPrediction = cv::imread(maskPath + filename + ".png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_UNCHANGED);
    //
    //                cv::imshow("loaded", maskedPrediction);
    //                cv::waitKey(0);
    //                //}
    //            }
    //        }
    //        cout << endl;
    //    }
    //    cout << endl;
}

void getBsParametersPerformance(cv::Mat overlaps, cv::Mat& means, cv::Mat& stddevs)
{
    //        BackgroundSubtractor<cv::BackgroundSubtractorMOG2, ColorDepthFrame>::Ptr pBs = pSys->getBackgroundSubtractor();
    //    means.create(overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
    //    stddevs.create(overlaps.rows, m_pBgSeq->getNumOfViews(), cv::DataType<float>::type);
    //
    //    int numOfFrames = overlaps.cols / m_pBgSeq->getNumOfViews();
    //    for (int v = 0; v < m_pBgSeq->getNumOfViews(); v++)
    //    {
    //        cv::Rect roi (v * numOfFrames, 0, numOfFrames, overlaps.rows);
    //        cv::Mat overlapsView (overlaps, roi);
    //
    //        for (int i = 0; i < overlaps.rows; i++)
    //        {
    //            cv::Scalar m, s;
    //            cv::meanStdDev(overlapsView.row(i), m, s);
    //            means.at<float>(i,v) = m.val[0];
    //            stddevs.at<float>(i,v) = s.val[0];
    //        }
    //    }
}