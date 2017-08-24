//
//  contour.cpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#include "contour.hpp"

void getBiggestContour(Mat const &src, vector<Point> &contour){
    assert(src.channels() == 1);
    imshow("contour2 src", src);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS, Point(0,0));
    
    vector<Point> skin_contour;
    
    // find biggest contour
    double biggest = 0;
    for( size_t i = 0; i < contours.size(); i++ )
    {
        double area = contourArea(contours[i]);
        if(area >= biggest){
            biggest = area;
            skin_contour = contours[i];
        }
    }
    contour = skin_contour;
    Mat dst = src.clone();
    drawContours(dst, contours, -1, blue);
    imshow("contour2", dst);
}
void drawContour(Mat const& src, vector<Point> &contour, Scalar color){
    Mat dst = src.clone();
    vector<vector<Point>> tempVecOfContour = {contour};
    drawContours(dst, tempVecOfContour, -1, color);
    imshow("contour", dst);
}
