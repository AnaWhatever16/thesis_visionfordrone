#include <vision/KCFTracker.h>

KCFTracker::KCFTracker(cv::Mat &_frame){

    tracker_ = cv::TrackerKCF::createTracker();

    while(!selectRef(_frame)){
        std::cout<<"Try again"<<std::endl;
    }
    std::cout<<"ROI selected"<<std::endl;

    std::vector<cv::Point2f> scene_corners(4);
    scene_corners = calcRoi();

    cv::Mat scene;
    // we take only the object we want to track
    scene =_frame(cv::Range(scene_corners[0].y, scene_corners[2].y), 
                    cv::Range(scene_corners[0].x, scene_corners[2].x)); 

    vecInit_ = computeCandidateLines(scene);
    frameInit_ = scene;
    angle = new AngleDetect(frameInit_, vecInit_);

    tracker_->init(_frame,roi_);
    printf("Start the tracking process, press ESC to quit.\n");

}

bool KCFTracker::selectRef(cv::Mat &_frame){
    roi_=cv::selectROI("tracker", _frame, true, false);
    if(roi_.width==0 || roi_.height==0){
        return 0;
    }
    return 1;
}

bool KCFTracker::update(cv::Mat &_frame){
    bool isfound = tracker_->update(_frame,roi_);
    if(!isfound){
        cv::waitKey(0);
        return 0;
    }
    //cv::rectangle(_frame, roi_, cv::Scalar(0, 255, 0), 2, 1);

    std::vector<cv::Point2f> scene_corners(4);
    scene_corners = calcRoi(); 

    cv::Mat scene;
    scene=_frame(cv::Range(scene_corners[0].y, scene_corners[2].y), 
                    cv::Range(scene_corners[0].x, scene_corners[2].x));

    std::vector<cv::Vec4i> vecActual;
    vecActual = computeCandidateLines(scene);
    anglePos_ = angle->detect(vecActual, scene);

    //cv::waitKey(3); //for debug in case we hace more images showing in windows

    std::cout << "Error angle: " << anglePos_ << std::endl;

    //Calculate centroid of reference
    cv::Moments mu;
    mu = moments(scene_corners);
    ref_= cv::Point2f (mu.m10/mu.m00, mu.m01/mu.m00);

    cv::circle(_frame, ref_, 1, cv::Scalar(0, 255, 0), 5);

    //Calculate image center 
    float cx = _frame.cols/2;
    float cy = _frame.rows/2;        
    imgCenter_= cv::Point2f(cx , cy);
    cv::circle(_frame, imgCenter_, 1, cv::Scalar(0, 0, 255), 5);

    cv::imshow("tracker",_frame);
    return 1;
}

std::vector<cv::Point2f> KCFTracker::calcRoi(){
    cv::Point2f ctl = roi_.tl();
    cv::Point2f cbr = roi_.br();
    cv::Size siz = roi_.size();
    cv::Point2f cbl = ctl + cv::Point2f(0, siz.height); 
    cv::Point2f ctr = ctl + cv::Point2f(siz.width, 0);

    std::vector<cv::Point2f> scene_corners(4);
    scene_corners[0]=ctl; scene_corners[1]= ctr;
    scene_corners[2]= cbr; scene_corners[3]= cbl;

    return scene_corners;
}

std::vector<cv::Vec4i> KCFTracker::computeCandidateLines(cv::Mat &_frame){
    // Convert image to grayscale
    cv::Mat gray;
    cv::cvtColor(_frame, gray, cv::COLOR_BGR2GRAY);

    // Convert image to binary
    cv::Mat bw, probMat;

    // Edge detection
    cv::Canny(gray, bw, 60, 190, 3);

    // Copy edges to the images that will display the results in BGR
    cv::cvtColor(bw, probMat, cv::COLOR_GRAY2BGR);

    // Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(bw, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    
    std::vector<cv::Vec4i> linePoints(linesP.size());
    for( size_t i = 0; i < linesP.size(); i++ ) {
        auto &pair = linesP[i];
        linePoints[i]  = { pair[0], pair[1], pair[2], pair[3] }; 
        cv::line( probMat, cv::Point(pair[0], pair[1]), cv::Point(pair[2], pair[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    
    std::vector<cv::Vec4i> vec_def;
    vec_def=defineLines(linePoints);

    return vec_def;
}

std::vector<cv::Vec4i> KCFTracker::defineLines(std::vector<cv::Vec4i> &_lines){
    std::vector<cv::Vec4i> vec_def;
    for (size_t i = 0; i<_lines.size(); i++){
        cv::Vec4i l;
        l = _lines[i];
        //define line as a point and a vector
        vec_def.push_back(cv::Vec4i(l[0],l[1],(l[2]-l[0]),(l[3]-l[1]))); 
    }
    return vec_def;
}