#include <vision/AngleDetect.h>

typedef Eigen::Matrix<float, 2, 3> Matrix2x3;

AngleDetect::AngleDetect(cv::Mat &_frame, std::vector<cv::Vec4i> &_vec){
    initFrame_ = _frame;
    vecInit_=_vec;
}

float AngleDetect::detect(std::vector<cv::Vec4i> &_vec, cv::Mat &_frame){
    float ang_w = 0;
    float bestScore = 1e5;
    //cv::Mat modif(_frame.rows,_frame.cols, CV_64F);
    cv::Mat blended;

    for(size_t i=0; i < vecInit_.size(); i++){
        cv::Vec4i ref;
        ref = vecInit_[i];
        cv::Vec2f vr(ref[2],ref[3]); //reference vector
        
        for (size_t j=0; j < _vec.size(); j++){
            cv::Vec4i obj = _vec[j];
            cv::Vec2f vo(obj[2],obj[3]); //objects vector

            float ang = (atan2(vr[1], vr[0]) - atan2(vo[1], vo[0]))*180/M_PI; // REF considered static.

            cv::Point center = cv::Point(_frame.cols/2, _frame.rows/2);
            float scale =1;
            cv::Mat rot_mat = getRotationMatrix2D( center, -ang, scale ); //we give the opposite angle because the matrix is opposite (see openCV doc)
            rot_mat.convertTo(rot_mat, CV_32F);

            cv::Mat mod_frame;
            cv::warpAffine(_frame, mod_frame, rot_mat, _frame.size());
            cv::Mat blend;
            addWeighted(initFrame_, 0.3, mod_frame, 0.5, 0.0, blend);
            float score = lineComparison(rot_mat, vecInit_, _vec, blend);
            
            
            cv::putText(blend, std::to_string(score), cv::Point(10,100), cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(255,255,255));
            std::stringstream ss;
            ss << "img_" << i << "_" << j << ".jpg";

            if (score < bestScore){
                bestScore = score;
                //modif=mod_frame; //debug
                //blended=blend; //debug
                ang_w=ang;
            }
        }
    }

    return ang_w;
}


float AngleDetect::lineComparison(cv::Mat &_T, std::vector<cv::Vec4i> &vecInit_, std::vector<cv::Vec4i> &_vec, cv::Mat &_blend){

    std::vector<Match> matches(_vec.size());
    std::vector<cv::Vec4i> rotatedObj(_vec.size());

    for(size_t i=0; i < _vec.size(); i++){
        cv::Mat p1 = cv::Mat(3,1,_T.type()); 
        p1.at<float>(0) = _vec[i][0]; 
        p1.at<float>(1) = _vec[i][1]; 
        p1.at<float>(2) = 1.0f;

        cv::Mat v = cv::Mat(2,1,_T.type()); 
        v.at<float>(0) = _vec[i][2]; 
        v.at<float>(1) = _vec[i][3]; 

        cv::Mat rp1 = _T*p1;

        cv::Mat rotMatrix = _T(cv::Rect(0,0,2,2));
        cv::Mat rv = rotMatrix*v;

        cv::Vec4i robj(  rp1.at<float>(0), rp1.at<float>(1), 
                        rv.at<float>(0), rv.at<float>(1));
        rotatedObj[i] = robj;
    }
    for(size_t i=0; i < _vec.size(); i++){
        float minScore = 1e5;
        for (size_t j=0; j < vecInit_.size(); j++){
            cv::Vec4i nref = vecInit_[j];
        
            float actualScore = scoreCalc(nref, rotatedObj[i]);
            // std::cout << actualScore << "\t";
            if (actualScore < minScore){   
                matches[i].id_line = j;
                matches[i].flag_eval = 0; //flag to evaluate later
                matches[i].score  = actualScore;   
                minScore = actualScore;         
            } //min score for each ref line
        }
    }

    return scoreSelection(matches); //see if there are not repeated object lines and add scores
}

cv::Vec3f AngleDetect::lineCalc (cv::Vec4i _vec){
    float m, y, n;

    if (_vec[2] == 0){ //vertical line
        m = 1;
        y = 0;
        n = _vec[0];
    }
    else{ //any other line
        m = _vec[3]/_vec[2];
        y = 1;
        n = _vec[1]-m*_vec[0];
    }
    return cv::Vec3f(m,y,n);
}

float AngleDetect::scoreCalc(cv::Vec4i &_ref, cv::Vec4i &_obj){
    cv::Vec3f rectRef = lineCalc(_ref);
    cv::Vec2f vr(_ref[2],_ref[3]); //objects vector
    cv::Vec3f rectObj = lineCalc(_obj);
    cv::Vec2f vo(_obj[2],_obj[3]); //objects vector

    Matrix2x3 Ab;
    Eigen::Matrix2f As;
    Ab <<   -rectRef[0], rectRef[1], rectRef[2], 
            -rectObj[0], rectObj[1], rectObj[2];
    As <<   -rectRef[0], rectRef[1], 
            -rectObj[0], rectObj[1];
    
    Eigen::FullPivLU<Matrix2x3> evalB(Ab);
    evalB.setThreshold(1e-5);
    Eigen::FullPivLU<Eigen::Matrix2f> evalS(As);
    evalS.setThreshold(1e-5);
    float ang, scoredist;
    if (evalB.rank() == evalS.rank()){
        if (evalB.rank() == 2){ //they intersect
            scoredist = 0;
            // std::cout << "vo: " << vo << std::endl;
            // std::cout << "vr " <<  vr << std::endl;
            ang = atan2(vo[1], vo[0]) - atan2(vr[1], vr[0]);
            //normalize it to the range [0, 2 π):
            //if (ang < 0) { ang += 2 * M_PI; } 
            ang=ang/M_PI; //normalized angle between [0,1]
        }
        else if(evalB.rank() == 1){ //they are the same line
            ang = 0;
            scoredist = 0;
        }
        else std::cout << "Something went terribly wrong" << std::endl;
    }
    else{ //they are parallel
        ang = 0;
        float dist = distanceCalc(_ref, _obj);
        // scoredist = 1 - 1/exp(dist); //dist=0 scoredist = 0; dist = inf scoredist = 1;
        scoredist = dist/initFrame_.cols;   // 666 
    }
    return (fabs(ang) + scoredist); //number should be between [0, 1] because one of them is always 0
}

float AngleDetect::scoreSelection(std::vector<Match> &_scores){

    float sumScore=0;
    float data = 0;
    for (size_t i = 0; i < _scores.size(); i++){
        if (_scores[i].flag_eval == 0){
            sumScore+=_scores[i].score;
            data++;
        }
    }

    if (data==0){
        std::cout << "No score" << std::endl; //for debug
        return 1e5;
    }
    else if (data < 4){
        //std::cout << "Not enough matches" << std::endl; //for debug
        return 1e5;
    }
    else{
        return sumScore/data;
    }

}

float AngleDetect::distanceCalc(cv::Vec4i &_ref, cv::Vec4i &_obj){
    cv::Vec2f pointRef  = {_ref[0], _ref[1]};
    cv::Vec2f pointObj  = {_obj[0], _obj[1]};

    float norm = sqrt(_ref[2]*_ref[2] + _ref[3]*_ref[3]);

    cv::Vec2f lineDirection = {_ref[2]/norm, _ref[3]/norm};
    
    float aux = (pointObj[0]-pointRef[0])*lineDirection[1] - (pointObj[1]-pointRef[1])*lineDirection[0];
    float mu = aux/(lineDirection[0]*lineDirection[0] + lineDirection[1]*lineDirection[1]);
    float dist = sqrt(mu*lineDirection[1]*mu*lineDirection[1] + mu*lineDirection[0]*mu*lineDirection[0]);

    return dist;
}