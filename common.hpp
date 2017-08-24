//
//  common.hpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#ifndef common_hpp
#define common_hpp

#include <stdio.h>
#include <list>
#include "opencv2/opencv.hpp"
#include "contour.hpp"
#include <opencv2/highgui.hpp>
#include "spline.hpp"
#include <time.h>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#define debugging false
#define showGuideLines false

using namespace cv;
using namespace std;

const int disiredWidth = 500;
const Scalar red = Scalar(0,0,255);
const Scalar blue = Scalar(255,0,0);

std::string random_string( size_t length );
void resizeImage(Mat &src);
void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle);
void blurImage(Mat &src);
float getDistance(Point a, Point b);
float getAngle(Point a, Point b);
Point getIntersect(Point x1, Point x2, Point y1, Point y2);
float isPointUpperToLine(Point target, Point lineA, Point lineB);
void fillArea(Mat&, vector<Point>, Scalar);
void makeSpline(vector<Point> const &points, vector<Point> &result);
//double meanOfArray(double* array, int size) {
//    double sum = 0.0;
//
//    for (int i = 0; i < size; i++)
//        sum += array[i];
//
//    return sum / size;
//};


//// 표준 편차 계산 함수
//double standardDeviation(double* array, int size, int option) {
//    // 배열 요소가 1개밖에 없을 때는
//    // NaN(숫자가 아님)이라는 의미로
//    // -1.#IND00 을 반환
//    if (size < 2) return sqrt(-1.0);
//
//    double sum = 0.0;
//    double sd = 0.0;
//    double diff;
//    double meanValue = mean(array, size);
//
//    for (int i = 0; i < size; i++) {
//        diff = array[i] - meanValue;
//        sum += diff * diff;
//    }
//    sd = sqrt(sum / (size - option));
//
//    return sd;
//};
template<typename Out>
void split(const std::string &s, char delim, Out result);
std::vector<std::string> split(const std::string &s, char delim);
void deleteDuplicateLines(vector<Vec4i> lines, vector<Vec4i>& result);
float getYRemain(Point x1, Point x2);
bool isPointOnTheLine(Point point, vector<Point> line);
#endif /* common_hpp */
