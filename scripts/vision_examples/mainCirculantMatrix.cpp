#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

int main(int _argc, char **_argv){

    cv::Mat frame = cv::imread(_argv[1], cv::IMREAD_COLOR);
    cv::Mat first = cv::Mat(frame, cv::Range(frame.rows/5, frame.rows), cv::Range(0, frame.cols)); 
    cv::Mat second = cv::Mat(frame, cv::Range(frame.rows*2/5, frame.rows), cv::Range(0, frame.cols));
    cv::Mat third = cv::Mat(frame, cv::Range(frame.rows*3/5, frame.rows), cv::Range(0, frame.cols));
    cv::Mat fourth = cv::Mat(frame, cv::Range(frame.rows*4/5, frame.rows), cv::Range(0, frame.cols));

    for (int i=0; i<frame.rows; i++){
        if(i < (frame.rows/5)){
            first.push_back(frame.row(i));
        }
        if(i < (frame.rows*2/5)){
            second.push_back(frame.row(i));
        }
        if(i < (third.rows*3/5)){
            third.push_back(frame.row(i));
        }
        if(i < (frame.rows*4/5)){
            fourth.push_back(frame.row(i));
        }        
    }       

    cv::imwrite("/home/ana/programming/thesis_visionfordrone/images/other/FirstPerm.jpg", first);
    cv::imwrite("/home/ana/programming/thesis_visionfordrone/images/other/SecondPerm.jpg", second);
    cv::imwrite("/home/ana/programming/thesis_visionfordrone/images/other/ThirdPerm.jpg", third);
    cv::imwrite("/home/ana/programming/thesis_visionfordrone/images/other/FourthPerm.jpg", fourth);


    return 0;
}