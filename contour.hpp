//
//  contour.hpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#ifndef contour_hpp
#define contour_hpp

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "common.hpp"

using namespace std;
using namespace cv;

void getBiggestContour(Mat const &src, vector<Point> &contour);
void drawContour(Mat const& src, vector<Point> &contour, Scalar color = Scalar(255,0,0));

#endif /* contour_hpp */
