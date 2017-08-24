//
//  common.cpp
//  opencv
//
//  Created by 김우현 on 10/07/2017.
//  Copyright © 2017 Woohyun Kim. All rights reserved.
//

#include "common.hpp"

void resizeImage(Mat &src) {
    float desiredWidthRatio = src.cols / disiredWidth;
    resize(src, src, Size(src.cols / desiredWidthRatio, src.rows / desiredWidthRatio), 0, 0, CV_INTER_LANCZOS4);
}

void blurImage(Mat &src) {
    GaussianBlur(src, src, Size(5, 5), 1.5);
}

float getDistance(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

float getAngle(Point a, Point b) {
    if (a.x == b.x) {
        return 999;
    }
    if (a.y == b.y) {
        return 0;
    }
    return ((float) b.y - (float) a.y) / ((float) a.x - (float) b.x);
}
std::string random_string( size_t length )
{
    srand((unsigned)time(NULL));
    auto randchar = []() -> char
    {
        const char charset[] =
                "0123456789"
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                        "abcdefghijklmnopqrstuvwxyz";
        const size_t max_index = (sizeof(charset) - 1);
        return charset[ rand() % max_index ];
    };
    std::string str(length,0);
    std::generate_n( str.begin(), length, randchar );
    return str;
}
float getYRemain(Point x1, Point x2) {
    float xAngle = -1 * getAngle(x1, x2);
    float xRemain = x1.y - xAngle * x1.x;
    return xRemain;
}

Point getIntersect(Point x1, Point x2, Point y1, Point y2) {
    float xAngle = -1 * getAngle(x1, x2);
    float yAngle = -1 * getAngle(y1, y2);
    float xRemain = x1.y - xAngle * x1.x;
    float yRemain = y1.y - yAngle * y1.x;
    float intersectX = (yRemain - xRemain) / (xAngle - yAngle);
    float intersectY = intersectX * xAngle + xRemain;
    if (cvRound(xAngle) != -1 * 999 && cvRound(yAngle) != -1 * 999) {
        return Point(cvRound(intersectX), cvRound(intersectY));
    } else if (cvRound(xAngle) == -1 * 999 && cvRound(yAngle) != -1 * 999) {
        return Point(x1.x, yAngle * x1.x + yRemain);
    } else if (cvRound(yAngle) == -1 * 999 && cvRound(xAngle) != -1 * 999) {
        return Point(y1.x, xAngle * y1.x + xRemain);
    } else if (cvRound(xAngle) == -1 * 999 && cvRound(yAngle) == -1 * 999) {
        return Point(0, 0);
    } else {
        return Point(0, 0);
    }
}

void drawExtendedLine(Mat &src, Point x1, Point x2) {
    float xAngle = -1 * getAngle(x1, x2);
    float xRemain = x1.y - xAngle * x1.x;
    if (xAngle == -1 * INT_MAX) {
        line(src, Point(x1.x, 0), Point(x1.x, src.cols), Scalar(0, 255, 0), 1, 8);
    } else {
        line(src, Point(0, cvRound(xRemain)), Point(src.cols, cvRound(src.cols * xAngle + xRemain)), Scalar(0, 255, 0),
             1, 8);
    }
}

float isPointUpperToLine(Point target, Point lineA, Point lineB) {
    float angle = getAngle(lineA, lineB);
    float b = lineA.y + lineA.x * angle;
    return target.x * angle + b > target.y;
}

void makeSpline(vector<Point> const &points, vector<Point> &result) {
    vector<double> X(points.size()), Y(points.size());

    for (int i = 0; i < points.size(); i++) {
        X[i] = points[i].x;
        Y[i] = points[i].y;

    }

    tk::spline s;
    s.set_points(X, Y);    // currently it is required that X is already sorted

    vector<Point> curve;

    for (int i = 0; i < 700; i++) {
        curve.push_back(Point(i, s(i)));
    }

    result = curve;

}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

void deleteDuplicateLines(vector<Vec4i> lines, vector<Vec4i> &result) {
    for (size_t i = 0; i < lines.size(); i++) {
        bool similar = false;
        for (size_t j = 0; j < lines.size(); j++) {
            if (i <= j) continue;
            Point x1 = Point(lines[i][0], lines[i][1]);
            Point x2 = Point(lines[i][2], lines[i][3]);
            Point y1 = Point(lines[j][0], lines[j][1]);
            Point y2 = Point(lines[j][2], lines[j][3]);

            if (
                    (abs(getAngle(x1, x2) - getAngle(y1, y2)) < 0.1 &&
                     abs(getYRemain(x1, x2) - getYRemain(y1, y2)) < 20)
                    ||
                    (abs(getAngle(x1, x2)) == 999 && abs(getAngle(y1, y2)) == 999)

                    ) {
                similar = true;
            }
        }
        if (similar) {
//            if (find(result.begin(), result.end(), lines[i])==result.end()){
//                // 없다면
//                result.push_back(lines[i]);
//            }
        } else {
            if (find(result.begin(), result.end(), lines[i]) == result.end()) {
                // 없다면
                result.push_back(lines[i]);
            }
        }
    }
}

bool isPointOnTheLine(Point point, vector<Point> line) {
    float standardDistance = getDistance(line[0], line[1]);
    return getDistance(point, line[0]) + getDistance(point, line[1]) < standardDistance + 2;
};

void fillArea(Mat &src, vector<Point> points4, Scalar color) {

}

void rotate_90n(cv::Mat const &src, cv::Mat &dst, int angle) {
    if (src.data != dst.data) {
        src.copyTo(dst);
    }

    angle = ((angle / 90) % 4) * 90;

    //0 : flip vertical; 1 flip horizontal
    bool const flip_horizontal_or_vertical = angle > 0 ? 1 : 0;
    int const number = std::abs(angle / 90);

    for (int i = 0; i != number; ++i) {
        cv::transpose(dst, dst);
        cv::flip(dst, dst, flip_horizontal_or_vertical);
    }
}