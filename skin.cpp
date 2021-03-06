//
//  skin.cpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#include "skin.hpp"


void getFootArea(Mat const &src, Mat& dst){
//    Mat skin;
//    cvtColor(src, skin, CV_BGR2YCrCb);
//    inRange(skin, Scalar(0,133,77), Scalar(255,173,127), skin);
    
//    dst = skin;
    
    Mat result;
    result = GetSkin(src);
    
    Mat mask = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3), cv::Point(1, 1));
    morphologyEx(result, result, cv::MORPH_DILATE, mask,Point(-1, -1) , 6);

    dst = result;
}

bool R1(int R, int G, int B) {
    bool e1 = (R>95) && (G>40) && (B>20) && ((max(R,max(G,B)) - min(R, min(G,B)))>15) && (abs(R-G)>15) && (R>G) && (R>B);
    bool e2 = (R>220) && (G>210) && (B>170) && (abs(R-G)<=15) && (R>B) && (G>B);
    return (e1||e2);
}

bool R2(float Y, float Cr, float Cb) {
    bool e3 = Cr <= 1.5862*Cb+20;
    bool e4 = Cr >= 0.3448*Cb+76.2069;
    bool e5 = Cr >= -4.5652*Cb+234.5652;
    bool e6 = Cr <= -1.15*Cb+301.75;
    bool e7 = Cr <= -2.2857*Cb+432.85;
    return e3 && e4 && e5 && e6 && e7;
}

bool R3(float H, float S, float V) {
    return (H<30) || (H > 220);
}

Mat GetSkin(Mat const &src) {
    // allocate the result matrix
    Mat dst = src.clone();
    
    Vec3b cwhite = Vec3b::all(255);
    Vec3b cblack = Vec3b::all(0);
    
    Mat src_ycrcb, src_hsv;
    // OpenCV scales the YCrCb components, so that they
    // cover the whole value range of [0,255], so there's
    // no need to scale the values:
    cvtColor(src, src_ycrcb, CV_BGR2YCrCb);
    // OpenCV scales the Hue Channel to [0,180] for
    // 8bit images, so make sure we are operating on
    // the full spectrum from [0,360] by using floating
    // point precision:
    src.convertTo(src_hsv, CV_32FC3);
    cvtColor(src_hsv, src_hsv, CV_BGR2HSV);
    // Now scale the values between [0,255]:
    normalize(src_hsv, src_hsv, 0.0, 255.0, NORM_MINMAX, CV_32FC3);
    
    for(int i = 0; i < src.rows; i++) {
        for(int j = 0; j < src.cols; j++) {
            
            Vec3b pix_bgr = src.ptr<Vec3b>(i)[j];
            int B = pix_bgr.val[0];
            int G = pix_bgr.val[1];
            int R = pix_bgr.val[2];
            // apply rgb rule
            bool a = R1(R,G,B);
            
            Vec3b pix_ycrcb = src_ycrcb.ptr<Vec3b>(i)[j];
            int Y = pix_ycrcb.val[0];
            int Cr = pix_ycrcb.val[1];
            int Cb = pix_ycrcb.val[2];
            // apply ycrcb rule
            bool b = R2(Y,Cr,Cb);
            
            Vec3f pix_hsv = src_hsv.ptr<Vec3f>(i)[j];
            float H = pix_hsv.val[0];
            float S = pix_hsv.val[1];
            float V = pix_hsv.val[2];
            // apply hsv rule
            bool c = R3(H,S,V);
            
            if(!((a&&b)&&c))
                dst.ptr<Vec3b>(i)[j] = cblack;
            else
                dst.ptr<Vec3b>(i)[j] = cwhite;

        }
    }
    return dst;
}
