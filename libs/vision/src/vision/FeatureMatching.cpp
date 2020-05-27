#include <vision/FeatureMatching.h>

FeatureMatching::FeatureMatching(std::string _argv){
    templ_ = cv::imread(_argv, CV_LOAD_IMAGE_GRAYSCALE);
    detector = cv::ORB::create( minHessian );
    /// Detection of template's features
    detection(templ_, keypointsTempl_, descriptorsTempl_);

    //Matcher without filter
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE);
    //Matcher with filter
    //matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
}

void FeatureMatching::method(cv::Mat &_input){
    /// Where keypoint detection of image occurs
    detection(_input, keypointsImg_, descriptorsImg_);
    /// Where matching between image and template occurs
    matching(_input);
}

void FeatureMatching::detection(cv::Mat &_input,std::vector<cv::KeyPoint> &_keypoints, cv::Mat &_descriptors)
{
    //-- Step 1: Detect the keypoints using ORB Detector
    detector->detectAndCompute(_input, cv::noArray(), _keypoints, _descriptors);
}

void FeatureMatching::matching(cv::Mat &_img){
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // ORB uses Hamming distance to determine features
    if (descriptorsImg_.rows>12){
        std::vector< std::vector<cv::DMatch> > knn_matches;
        matcher->knnMatch(descriptorsTempl_, descriptorsImg_, knn_matches, 2 );
        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f; //0.77f
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        //-- Draw matches
        cv::Mat img_matches;
        drawMatches( templ_, keypointsTempl_, _img, keypointsImg_, good_matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        drawBoundBox(good_matches, img_matches);

        float cx = (img_matches.cols-templ_.cols)/2 + templ_.cols;
        float cy = img_matches.rows/2;
        imgCenter_= cv::Point2f(cx , cy);
        cv::circle(img_matches, imgCenter_, 1, cv::Scalar(0, 0, 255), 5);

        imshow("Resultado", img_matches );
    }
    else{
        imshow("Resultado", _img);
    }
}
void FeatureMatching::drawBoundBox(std::vector<cv::DMatch> &_good_matches, cv::Mat &_imgMatches){
    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for( int i = 0; i < _good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        scene.push_back( keypointsImg_[ _good_matches[i].trainIdx ].pt );
    }

    std::vector<cv::Point2f> hull;
    cv::convexHull(scene, hull, true);
    cv::Rect bound = cv::boundingRect(hull);
    cv::Point2f ctl = bound.tl();
    cv::Point2f cbr = bound.br();
    cv::Size siz = bound.size();
    cv::Point2f cbl = ctl + cv::Point2f(0, siz.height); 
    cv::Point2f ctr = ctl + cv::Point2f(siz.width, 0);

    std::vector<cv::Point2f> scene_corners(4);
    scene_corners[0]=ctl; scene_corners[1]= ctr;
    scene_corners[2]= cbr; scene_corners[3]= cbl;

    rectangle( _imgMatches, ctl + cv::Point2f( templ_.cols, 0), cbr + cv::Point2f( templ_.cols, 0), cv::Scalar(0, 255, 0), 4);

    //Calculate centroid
    cv::Moments mu;
    mu = moments(scene_corners);
    cv::Point2f c(mu.m10/mu.m00, mu.m01/mu.m00);

    ref_= c + cv::Point2f( templ_.cols, 0);

    cv::circle(_imgMatches, ref_, 1, cv::Scalar(0, 0, 0), 5);
}