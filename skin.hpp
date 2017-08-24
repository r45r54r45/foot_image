//
//  skin.hpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#ifndef skin_hpp
#define skin_hpp

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "contour.hpp"

using namespace std;
using namespace cv;

void getFootArea(Mat const &src, Mat& dst);
bool R1(int R, int G, int B);
bool R2(float Y, float Cr, float Cb);
bool R3(float H, float S, float V);
Mat GetSkin(Mat const &src);

#endif /* skin_hpp */

