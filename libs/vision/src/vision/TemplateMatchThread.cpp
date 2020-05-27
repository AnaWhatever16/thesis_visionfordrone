#include <vision/TemplateMatchThread.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <iostream>

TemplateMatchThread::TemplateMatchThread(std::string _argv){
    templ_ = cv::imread(_argv, CV_LOAD_IMAGE_COLOR);

    for (int i=0; i<5;i++){
        cv::Mat scaledTemplate;
        resize(templ_, scaledTemplate, cv::Size(), (1-i*0.20), (1-i*0.20));
        templResize_.push_back(scaledTemplate.clone());
    }
}

TemplateMatchThread::TemplateMatchThread(cv::Mat &_frame){
    templ_ = _frame;

    for (int i=0; i<3;i++){
        cv::Mat scaledTemplate;
        resize(templ_, scaledTemplate, cv::Size(), (1-i*0.45), (1-i*0.45));
        templResize_.push_back(scaledTemplate.clone());
    }
}

double TemplateMatchThread::matchThread(cv::Mat &_input, int _matchMethod){
    std::vector<std::thread> vt(templResize_.size());
    std::vector<double> maxScore(templResize_.size());
    std::vector<double> minScore(templResize_.size());
    std::vector<cv::Point> maxScoreLoc(templResize_.size());
    std::vector<cv::Point> minScoreLoc(templResize_.size());
    std::vector<int> col(templResize_.size());
    std::vector<int> row(templResize_.size());

    cv::Mat img_display;
    _input.copyTo(img_display);
    double minVal_act=800;
    double maxVal_act=0;
    cv::Point matchLoc;
    int cols, rows;

    //this are now given through function
    //int match_method=cv::TM_SQDIFF_NORMED;
    //int match_method=cv::TM_CCORR;
    //int match_method=cv::TM_CCOEFF;
    //int match_method=cv::TM_CCOEFF_NORMED;


    for (int i=0;i<templResize_.size();i++){
        vt[i]=std::thread([&](int _id){
            cv::Mat eval = templResize_[_id];

            //int result_cols = _input.cols - eval.cols + 1;
            //int result_rows = _input.rows - eval.rows + 1;
            cv::Mat result;//(result_rows, result_cols, CV_32FC1);

            //function where the template is matched with the camera image
            matchTemplate(_input, eval, result, _matchMethod);
            // normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

            double minVal, maxVal;       
            cv::Point minLoc, maxLoc;
            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

            maxScore[_id]=maxVal; maxScoreLoc[_id]=maxLoc;
            minScore[_id]=minVal; minScoreLoc[_id]=minLoc;
            col[_id]=eval.cols; row[_id]=eval.rows;
        }, i);
    }
    
    for (int i=0; i<templResize_.size();i++){
        while(!vt[i].joinable()){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        vt[i].join();
    }

    const char *image_window = "Source Image";
    
    for(int i=0;i<templResize_.size();i++){
        if(_matchMethod==cv::TM_SQDIFF_NORMED){
            if (minScore[i]<minVal_act) {
                minVal_act=minScore[i];
                matchLoc = minScoreLoc[i]; 
                cols=col[i];
                rows=row[i];
            }
        }
        else{
            if (maxScore[i]>maxVal_act) {
                maxVal_act=maxScore[i];
                matchLoc = maxScoreLoc[i]; 
                cols=col[i];
                rows=row[i];
            }
        }
    }
    
    //cvtColor(img_display, img_display, CV_RGB2BGR); //only for autopilot
    rectangle(img_display, matchLoc, cv::Point(matchLoc.x + cols, matchLoc.y + rows), cv::Scalar::all(0), 2, 8, 0);
    imshow(image_window, img_display);

    return maxVal_act;
}