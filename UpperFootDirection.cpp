//
//  UpperFootDirection.cpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#include "UpperFootDirection.hpp"
#include "skin.hpp"
#include <time.h>

Mat src;

string fileName;

void analyzeUpperFootDirection(Mat const sourceImg ,int& footWidth, string& fileName, vector<int>& pointsResult) {
    // preparing image
    src = sourceImg;
//    rotate(src,  90, src);
    Mat greySrc;
    cvtColor(src, greySrc, CV_BGR2GRAY);

    Mat thresholdDst;
    adaptiveThreshold(greySrc, thresholdDst, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 19, 7);

    Mat mask2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    morphologyEx(thresholdDst, thresholdDst, cv::MORPH_DILATE, mask2, Point(-1, -1), 1);
#if(debugging)
    imshow("thresholdDst", thresholdDst);
#endif
    vector<Point> point5;
    getFootOuter5Points(thresholdDst, point5);

    for (int i = 0; i < point5.size(); i++) {
        circle(src, point5[i], 3, red);
        pointsResult.push_back(src.rows - point5[i].y);
    }
    if(point5.size() == 5){
        vector<Point> spline;
        makeSpline(point5, spline);

        for (int i = 0; i < spline.size(); i++) {
            circle(src, spline[i], 1, blue);
        }
    }

    morphologyEx(thresholdDst, thresholdDst, cv::MORPH_DILATE, mask2, Point(-1, -1), 1);
    getBiggestFootWidth(thresholdDst, footWidth, src);

    fileName = "";
    string path = "../server/resultImage/";
    fileName.append(random_string(10));
    fileName.append(".jpg");
#if(debugging)
    imshow("result", src);
#endif
    imwrite(path + fileName, src);
}

#define showImg2 false

void getBiggestFootWidth(Mat const &src, int &pixel, Mat& draw) {
    int max = 0;
    Point maxLeft;
    Point maxRight;
    //TODO 임시방편 -20 해주기
    for (int i = src.rows-20; i > 0; i--) {
        int leftMost = 0;
        int rightMost = 0;
        for (int j = 0; j < src.cols; j++) {
            if (src.at<uchar>(i,j) == 0) {
                if (leftMost == 0) {
                    leftMost = j;
                } else {
                    rightMost = j;
                }
            }

        }
        int diff = rightMost - leftMost;
        if (diff > max) {
            max = diff;
            maxLeft = Point(leftMost, i);
            maxRight = Point(rightMost, i);
        }
    }
    line(draw, maxLeft, maxRight, blue, 3);
    pixel = max;
}


void getFootInner4Points(Mat const &src, vector<Point> &points, vector<Point> outer5Points) {
    Mat cpy;
    src.copyTo(cpy);


#if(showImg2)

    Mat upperOutline = src.clone();

#endif

    vector<Mat> rois;
    for (int i = 0; i < 4; i++) {
        Mat part = cpy(Rect(outer5Points[i].x, outer5Points[i].y, outer5Points[i + 1].x - outer5Points[i].x, 200));
        //TODO 200 바꿔야됨
        rois.push_back(part);
#if(debugging)
//        imshow("rois" + to_string(i), part);
#endif
    }

    vector<Point> result;

    for (int i = 0; i < 4; i++) {
        Mat roi = rois[i];
        for (int j = roi.rows - 1; j > -1; j--) {
            bool meet = false;
            for (int k = 0; k < roi.cols; k++) {
                if (roi.at<uchar>(j, k) == 0) {
                    meet = true;
                    result.push_back(Point(outer5Points[i].x + k, outer5Points[i].y + j));
                    break;
                }
            }
            if (meet) {
                break;
            }
        }

    }


    points = result;


#if(showImg2)
//    imshow("graph2", upperOutline);
#endif


}

void getFootOuter5Points(Mat const &src, vector<Point> &points) {
    assert(src.channels() == 1);

#if(debugging)

    Mat upperOutline = Mat::zeros(src.rows, src.cols, src.type());
    upperOutline.setTo(255);

#endif
    // 위에서 아래쪽으로 내려다 볼때 가장 먼저 만나는 점을 기록 (row 방향으로 0부터 700까지 swip)
    vector<Point> upperOutlinePoints;
#if(debugging)
    Mat testImg = upperOutline.clone();
#endif
    for (int i = 0; i < src.cols; i++) {
        for (int j = 0; j < src.rows; j++) {
            if (src.at<uchar>(j, i) == 0) {
#if(debugging)
                upperOutline.at<uchar>(j, i) = 0;
                testImg.at<uchar>(j, i) = 0;
#endif
                upperOutlinePoints.push_back(Point(i, j));
                break;
            }
        }
    }
//    imshow("testing", testImg);
#if(debugging)
    upperOutline.setTo(255);
#endif

    // row 방향으로 오른쪽으로 움직이면서 앞 뒤 점 사이의 간격이 3이 넘으면 솎아낸다 (혼자 떨어져있는 점을 없앤다)
    vector<Point> scopedUpperOutlinePoints;
    for (int i = 1; i < upperOutlinePoints.size() - 1; i++) {
        Point prev = upperOutlinePoints[i - 1];
        Point current = upperOutlinePoints[i];
        Point post = upperOutlinePoints[i + 1];

        if ((getDistance(prev, current) + getDistance(current, post)) / 2 < 3) {
            scopedUpperOutlinePoints.push_back(current);
#if(debugging)
            upperOutline.at<uchar>(current.y, current.x) = 0;
#endif
        }
    }
#if(debugging)
    upperOutline.setTo(255);
#endif

    // row 방향으로 오른쪽으로 움직이면서 앞 뒤 점의 y방향 좌표가 현재 점과 2이상 차이나는 애를 솎는다. (급격하게 높이가 변하는 점을 없앤다)
    vector<Point> peakUpperOutlinePoints;
    for (int i = 1; i < scopedUpperOutlinePoints.size() - 1; i++) {
        Point prev = scopedUpperOutlinePoints[i - 1];
        Point current = scopedUpperOutlinePoints[i];
        Point post = scopedUpperOutlinePoints[i + 1];

        if (abs(prev.y - current.y) < 3 && abs(current.y - post.y) < 3) {
#if(debugging)
            upperOutline.at<uchar>(current.y, current.x) = 0;
#endif
            peakUpperOutlinePoints.push_back(current);
        }
    }


#if(debugging)
    upperOutline.setTo(255);
#endif

    // 후보 클러스터를 구한다
    // 현재 점과 다음점을 비교해서 같은 발가락을 나타내는 점인지 확인한다

    vector<vector<Point>> footPointClusterCandidates;
    bool clusterStarted = false;
    vector<Point> tempCluster;
    float previousDelta = 0;
    for (int i = 0; i < peakUpperOutlinePoints.size() - 1; i++) {
        Point current = peakUpperOutlinePoints[i];
        Point post = peakUpperOutlinePoints[i + 1];
        // 다음 점이 현재점과 x축기준으로 바로 오른쪽에 있고, 높이 차이가 2이상 나지 않는다면 같은 클러스터로 생각한다

        if (post.x == current.x + 1 && abs(abs(post.y - current.y) - previousDelta) < 2) {
            if (!clusterStarted) {
                // 현재 점이 기존에 만들어지고 있던 클러스터의 일원이 아니라면 새로운 클러스터의 시작을 의미한다
                clusterStarted = true;
                tempCluster = {};
                tempCluster.push_back(current);
            } else if (clusterStarted && i == peakUpperOutlinePoints.size() - 2) {
                // 현재 점이 기존에 만들어지고 있던 클러스터의 일원이고, 마지막 점 바로 왼쪽 점이라면 같은 클러스터로 생각하고 클러스터 만들기를 종료한다.
                tempCluster.push_back(current);
                footPointClusterCandidates.push_back(tempCluster);
            } else {
                tempCluster.push_back(current);
            }
        } else {
            // 만약 클러스터가 만들어지고 있었다면
            if (clusterStarted) {
                // 클러스터의 종료를 의미한다
                clusterStarted = false;
                tempCluster.push_back(current);
                footPointClusterCandidates.push_back(tempCluster);
            }
        }

        previousDelta = abs(post.y - current.y);
    }


    vector<vector<Point>> mergedFootPointClusterCandidates;
    // 클러스터사이의 길이를 기준으로 합치거나 없애줌
    // TODO 클러스터의 구성 커브가 많을 때도 하나로 합쳐줌
    bool mergedBefore = false;
    vector<Point> tempVecPoints;

    for (int i = 0; i < footPointClusterCandidates.size() - 1; i++) {
        vector<Point> cluster = footPointClusterCandidates[i];
        //길이 확인 3개 이하면 제외
        if (cluster.size() < 3)continue;
        Point lastPoint = cluster[cluster.size() - 1];
        Point nextFirstPoint = footPointClusterCandidates[i + 1][0];

        if (getDistance(lastPoint, nextFirstPoint) < 10 && abs(lastPoint.y - nextFirstPoint.y) < 10) {
            //거리가 가까운 경우 합쳐준다
            if (mergedBefore) {
                tempVecPoints.insert(tempVecPoints.end(), footPointClusterCandidates[i + 1].begin(),
                                     footPointClusterCandidates[i + 1].end());
            } else {
                tempVecPoints = cluster;
                tempVecPoints.insert(tempVecPoints.end(), footPointClusterCandidates[i + 1].begin(),
                                     footPointClusterCandidates[i + 1].end());
                mergedBefore = true;
            }
        } else {
            // 거리가 먼경우 넘어가준다
            if (mergedBefore) {
                mergedFootPointClusterCandidates.push_back(tempVecPoints);
                mergedBefore = false;
            } else {
                mergedFootPointClusterCandidates.push_back(cluster);
                mergedBefore = false;
            }
        }
    }

    Mat temptemp = src.clone();
    temptemp.setTo(255);


    for (int i = 0; i < mergedFootPointClusterCandidates.size(); i++) {
        vector<Point> cluster = mergedFootPointClusterCandidates[i];
        for (int j = 0; j < cluster.size(); j++) {
            Point current = cluster[j];
            temptemp.at<uchar>(current.y, current.x) = 0;
        }
    }

//    imshow("temptemp", temptemp);


    vector<long> clusterSize;

    for (int i = 0; i < mergedFootPointClusterCandidates.size(); i++) {
        vector<Point> cluster = mergedFootPointClusterCandidates[i];
        clusterSize.push_back(cluster.size());
    }

    // 클러스터를 사이즈 순으로 정렬한다
    sort(clusterSize.begin(), clusterSize.end(), greater<int>());


    vector<vector<Point>> cleanedFootPointClusterCandidates;

    // 클러스터의 개수가 5개를 넘는다면 몇개를 솎아내 줘야한다
    if (mergedFootPointClusterCandidates.size() > 5) {
        // 5번째로 큰 클러스터의 크기
        long sizeBorder = clusterSize.at(4);
        // 해당 크기보다 같거나 큰 클러스트로 솎아낸다
        for (int i = 0; i < mergedFootPointClusterCandidates.size(); i++) {
            vector<Point> cluster = mergedFootPointClusterCandidates[i];
            if (cluster.size() >= sizeBorder) {
                cleanedFootPointClusterCandidates.push_back(cluster);
            }
        }
        // TODO 길이 순으로 잡았을시 발가락 한개가 적은 부분 잡혔을 경우 씹힘


    } else if (mergedFootPointClusterCandidates.size() < 5) {
//        cerr << "cluster detected count under 5" << endl;
        cleanedFootPointClusterCandidates = mergedFootPointClusterCandidates;
    } else {
        cleanedFootPointClusterCandidates = mergedFootPointClusterCandidates;
    }


    vector<Point> points5;

    for (int i = 0; i < cleanedFootPointClusterCandidates.size(); i++) {
        vector<Point> cluster = cleanedFootPointClusterCandidates[i];
        Point localHigh = Point(1000, 1000);
        for (int j = 0; j < cluster.size(); j++) {
            Point current = cluster[j];
            // y축 방향으로 값이 제일 작은 점을 찾아서 local high point 라 한다
            if (current.y < localHigh.y) {
                localHigh = current;
            }
#if(debugging)
            putText(upperOutline, to_string(i), cluster[0], FONT_HERSHEY_PLAIN, 1.0, 0, 2.0);
            upperOutline.at<uchar>(current.y, current.x) = 0;
#endif
        }
        points5.push_back(localHigh);
    }

//    assert(points5.size() == 5);
    points = points5;

#if(debugging)
//    imshow("graph", upperOutline);
#endif

    // 의도치 않게 되는 코드
//    for(int i =2 ; i<src.cols -2; i ++){
//        for(int j =2; j<src.rows -2; j++){
//            bool downsideNoDot = true;
//            for(int k=-2; k<3; k++){
//                if(src.at<uchar>(j-1,i+k) == 0 || src.at<uchar>(j-2,i+k) == 0){
//                    downsideNoDot = false;
//                    break;
//                }
//            }
//            if(!downsideNoDot){
//                break;
//            }else{
//                int blackCount = 0;
//                for(int k=-2; k<3; k++){
//                    if(src.at<uchar>(j+1,i+k) == 0 || src.at<uchar>(j+2,i+k) == 0){
//                        blackCount ++;
//                    }
//                }
//                if(blackCount > 1 && blackCount < 3){
//#if(showImg)
//                    upperOutline.at<uchar>(j,i) = 0;
//#endif
//                }
//            }
//        }
//    }

}
