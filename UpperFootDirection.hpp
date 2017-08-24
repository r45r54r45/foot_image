//
//  UpperFootDirection.hpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#ifndef UpperFootDirection_hpp
#define UpperFootDirection_hpp

#include <stdio.h>
#include "common.hpp"

void analyzeUpperFootDirection(Mat const sourceImg ,int& footWidth, string& fileName, vector<int>&);
void getBiggestFootWidth(Mat const &src, int& pixel, Mat &draw);
void getFootOuter5Points(Mat const &src, vector<Point> &points);
void getFootInner4Points(Mat const &src, vector<Point> &points, vector<Point> outer5Points);

#endif /* UpperFootDirection_hpp */
