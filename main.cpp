#pragma warning(disable: 4819)

#include "opencv2/opencv.hpp"
#include <iostream>
#include "opencv2/core/ocl.hpp"
#include <math.h>
#include "skin.hpp"
#include "UpperFootDirection.hpp"
#include "CardDetector.h"
using namespace cv;
using namespace std;

Mat resizedSrc;
void drawRect(Mat &img, vector<Point> &contours);
void setSrcQuad2(vector<Point> corners, Point2f srcQuad[4]);
void getSkin(Mat& src, Mat& dst);
vector<Point> getContour(Mat& src);
Point cutUpperAnkle(Mat& src, Mat& dst, vector<Point> contour);
vector<Point> getBiggestContour(Mat& src);
void makeCircle(Mat& src, Mat& dst, vector<Point> contour, Point);
vector<Point> findIntersects(vector<Point> contour, Point center, float radius);
vector<Point> findMaxDistPointBetweenLineAndPoint(Point lineStart, Point lineEnd, vector<Point> targetPoints);
vector<Point> findPointsLowerToLine(Point lineStart, Point lineEnd, vector<Point> targetPoints);
vector<Point> getContourPointsBetween(Point a, Point b, vector<Point> contour);
float distanceBetween(Point a, Point b);

Scalar white = Scalar(255,255,255);
Scalar green = Scalar(40,255,0);
Scalar purple = Scalar(40,255,128);

int main(int argc, char** argv)
{
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    bool isCardDetected;
    float cardRatio;
    Mat imgWithoutCard;
    clock_t begin = clock();
    analyzeCard(argv[1], isCardDetected, cardRatio, imgWithoutCard);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
//    cout << "time: "<<elapsed_secs*1000 <<"ms" << endl;

    if(!isCardDetected){
        cout << "{\"error\":\"card detection failure\"}" << endl;
        return 0;
    }

    int footWidthPixel;
    string fileName;
    vector<int> points;
    try{
        analyzeUpperFootDirection(imgWithoutCard, footWidthPixel, fileName, points);
    }catch (Exception exception1){

    }
    float footWidth = footWidthPixel * cardRatio;
    string result;
    result.append("{");
    result.append("\"width\":");
    result.append(to_string(footWidth));
    result.append(",");
    result.append("\"filename\":");
    result.append("\""+fileName+"\"");
    result.append(",");
    result.append("\"points\":");
    result.append("[");
    for(int i=0; i<points.size() ; i++){
        if(i!=points.size()-1){
            result.append(to_string(points[i])+",");
        }else{
            result.append(to_string(points[i]));
        }
    }
    result.append("]");
    result.append("}");

    cout << result <<endl;



    //
    //#define img 1
    //#if(img == 1)
    //    Mat src = imread("foot4.JPG", IMREAD_COLOR);
    //#else
    //    Mat src = imread("foot3.JPG", IMREAD_COLOR);
    //#endif
    //        resize(src, resizedSrc, Size(src.cols / 6, src.rows / 6 ), 0, 0, CV_INTER_LANCZOS4);
    //
    //        GaussianBlur(resizedSrc, resizedSrc, Size(7,7), 1.5);
    //    Mat dst = resizedSrc.clone();
    //    getFootArea(resizedSrc, dst);
    //    imshow("src", resizedSrc);
    //
    //    imshow("footArea", dst);
    //
    ////
    ////    resize(src, resizedSrc, Size(src.cols / 6, src.rows / 6 ), 0, 0, CV_INTER_LANCZOS4);
    ////
    ////    GaussianBlur(resizedSrc, resizedSrc, Size(7,7), 1.5);
    ////
    ////    imshow("original", resizedSrc);
    ////
    ////    Mat skin2 = GetSkin(resizedSrc);
    ////    imshow("slin2",skin2);
    ////
    ////    Mat skin;
    ////    getSkin(resizedSrc, skin);
    ////    imshow("skin", skin);
    ////
    ////    Mat withoutAnkle;
    ////    Point anklePoint = cutUpperAnkle(skin,withoutAnkle, getBiggestContour(skin));
    ////    imshow("without Ankle", withoutAnkle);
    ////
    ////
    ////    Mat tmp;
    ////    Mat tmp2 = 255 - withoutAnkle;
    ////
    ////    makeCircle(withoutAnkle, tmp, getBiggestContour(tmp2), anklePoint);
    ////
#if(debugging)
    waitKey(0);
#endif
    return 0;


}
void makeCircle(Mat& src, Mat& dst, vector<Point> contour, Point anklePos){
    Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
    src.copyTo(dst);
    
    vector<Point> contour_poly;
    Point2f center;
    float radius =0;
    
    approxPolyDP( Mat(contour), contour_poly, 3, true );
    
    minEnclosingCircle(contour_poly, center, radius );
    cout << "minEnclosingCircle => center: " << center << ", radius: " << radius << endl;
    
    vector<vector<Point> > contours_poly_vec;
    contours_poly_vec = {contour};
    drawContours( drawing, contours_poly_vec, -1, white, 1, 8, vector<Vec4i>(), 0, Point() );
    //    circle( drawing, center, radius, red, 2, 8, 0 );
    
    vector<Point> intersects = findIntersects(contour, center, radius);
    for(int i =0; i<intersects.size(); i++){
        circle( drawing, intersects[i], 3, white, 2, 8, 0 );
    }
    
    circle( drawing, center, 3, white, 2, 8, 0 );
    
    circle( drawing, anklePos, 3, white, 2, 8, 0 );
    
    line(drawing, intersects[0], intersects[1],red);
    
    vector<Point> lowerContour = findPointsLowerToLine(intersects[0], intersects[1], contour);
    
    vector<Point> lowest = findMaxDistPointBetweenLineAndPoint(intersects[0], intersects[1],lowerContour);
    
    cout <<lowest << endl;
    circle( drawing, lowest[1], 3, white, 2, 8, 0 );
    
    line(drawing, lowest[0], lowest[1], blue);
    
    Point intersect90degreePoint = lowest[0];
    
    vector<Point> rightBottomContour = getContourPointsBetween(intersects[1], lowest[1], lowerContour);
    
    vector<Point> result = findMaxDistPointBetweenLineAndPoint(intersects[1], lowest[1],rightBottomContour);
    
    line(drawing, intersects[1], lowest[1], green);
    
    circle( drawing, result[1], 3, white, 2, 8, 0 );
    
    line(drawing, intersects[0], result[1], purple);
    
    string dist =  to_string(distanceBetween(intersects[0], result[1]));
    
    putText( drawing, dist , Point(10,100), FONT_HERSHEY_SIMPLEX, 1, red, 2, 8);
    
    Point B = intersects[0];
    Point A = anklePos;
    Point C = result[1];
    float c = distanceBetween(A,B );
    float a = distanceBetween(B, C);
    float b = distanceBetween(A, C);
    float cosB = (pow(a,2)+pow(c,2)-pow(b,2))/(2*a*c);
    cout << "cosB: " << cosB << endl;
    float degree = acos(cosB);
    
    
    line(drawing, A, B, purple);
    line(drawing, A, C, purple);
    
    cout << "degree: " << degree*180/M_PI << endl;
    
    putText( drawing, to_string(degree*180/M_PI) , Point(10,200), FONT_HERSHEY_SIMPLEX, 1, red, 2, 8);
    
//    imshow("draw", resizedSrc + drawing);
    
}

Point cutUpperAnkle(Mat& src, Mat& dst, vector<Point> contour){
    Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
    src.copyTo(dst);
    
    vector<Point> contour_poly;
    Point2f center;
    float radius =0;
    
    approxPolyDP( Mat(contour), contour_poly, 3, true );
    
    minEnclosingCircle(contour_poly, center, radius );
    cout << center << radius << endl;
    
    vector<vector<Point>> contours_poly_vec = {contour};
    drawContours( drawing, contours_poly_vec, -1, white, 1, 8, vector<Vec4i>(), 0, Point() );
    circle( drawing, center, radius, red, 2, 8, 0 );
    
    
    // 원의 중심에서 가장 가까운점으로 발목부분을 추정
    float min = 99999.0;
    Point minDistPoint;
    for(int i =0; i<contour_poly.size(); i++){
        float computedDist = sqrt(pow(center.y - contour_poly[i].y,2)+pow(center.x - contour_poly[i].x,2));
        if(computedDist < min){
            min = computedDist;
            minDistPoint =contour_poly[i];
        }
    }
    
    circle( drawing, minDistPoint, 3, red, 2, 8, 0 );
    
//    imshow("tmp",drawing);
    //    발목 부분을 0 값으로 채워 지워줌
    for(int i = minDistPoint.y; i>-1; i--){
        for(int j = 0; j < src.cols; j++){
            dst.at<uchar>(i,j) = 0;
        }
    }
    dst = 255 - dst;
    
    return minDistPoint;
}
vector<Point> getBiggestContour(Mat& src){
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
    return skin_contour;
}

void getSkin(Mat& src, Mat& dst){
    Mat skin;
    cvtColor(src, skin, CV_BGR2YCrCb);
    inRange(skin, Scalar(0,133,77), Scalar(255,173,127), skin);

    Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(1, 1));

    morphologyEx(skin, skin, cv::MORPH_OPEN, mask,Point(-1, -1) , 2);
#if(debugging)

//    imshow("skin", skin);
#endif
    dst = skin;
}

vector<Point> findIntersects(vector<Point> contour, Point center, float radius){
    vector<Point> intersects = {};
    for(int i=0; i< contour.size(); i++){
        float computedDist = sqrt(pow(center.y - contour[i].y,2)+pow(center.x - contour[i].x,2));
        if(abs(computedDist - radius) < 1){
            intersects.push_back(contour[i]);
        }
    }
    
    vector<Point> cleanedIntersects = {};
    
    for(int i =0; i< intersects.size(); i++){
        Point current =intersects[i];
        if(cleanedIntersects.size() == 0){
            cleanedIntersects.push_back(current);
            continue;
        }
        bool isCloseExists = false;
        for(int j =0; j< cleanedIntersects.size(); j++){
            Point test =cleanedIntersects[j];
            float distance = sqrt(pow(test.x - current.x,2) + pow(test.y - current.y,2));
            if(distance < 50){
                isCloseExists = true;
            }
        }
        if(!isCloseExists){
            cleanedIntersects.push_back(current);
        }
    }
    
    cout <<cleanedIntersects ;
    return cleanedIntersects;
}
vector<Point> findMaxDistPointBetweenLineAndPoint(Point lineStart, Point lineEnd, vector<Point> targetPoints){
    float slope = ((float)lineStart.y - lineEnd.y) / ((float)lineStart.x - lineEnd.x);
    float targetSlope = -1/slope;
    float b = lineEnd.y - lineEnd.x * slope;
    
    float max = 0;
    Point maxPoint;
    vector<Point> result ;
    
    for(int i=0; i<targetPoints.size(); i++){
        int x = targetPoints[i].x;
        int y = targetPoints[i].y;
        
        float targetB = y-(x*targetSlope);
        float targetX = (float)(b - targetB) / (targetSlope - slope);
        float targetY = targetSlope * targetX + targetB;
        
        float dist = sqrt(pow(targetX - x,2)+pow(targetY - y,2));
        if(dist > max){
            max = dist;
            maxPoint = targetPoints[i];
            result = { Point(targetX,targetY),maxPoint};
        }
    }
    return result;
}
vector<Point> findPointsLowerToLine(Point lineStart, Point lineEnd, vector<Point> targetPoints){
    float slope = ((float)lineStart.y - lineEnd.y) / ((float)lineStart.x - lineEnd.x);
    cout <<"slope : "<< slope << endl;
    
    float b = lineEnd.y - lineEnd.x * slope;
    
    vector<Point> result = {};
    
    for(int i=0; i<targetPoints.size(); i++){
        int x = targetPoints[i].x;
        int y = targetPoints[i].y;
        
        if(x*slope + b < y){
            result.push_back(targetPoints[i]);
        }
    }
    return result;
    
}

vector<Point> getContourPointsBetween(Point a, Point b, vector<Point> contour){
    vector<Point> result;
    bool started = false;
    for(int i=0; i<contour.size(); i++){
        if(contour[i] == a || contour[i] == b){
            started = !started;
        }
        if(!started){
            continue;
        }else{
            result.push_back(contour[i]);
        }
    }
    return result;
}
float distanceBetween(Point a, Point b){
    return sqrt(pow(a.x - b.x,2)+pow(a.y - b.y,2));
}














