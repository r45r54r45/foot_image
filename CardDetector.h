//
// Created by 김우현 on 28/07/2017.
//

#ifndef MAINPROGRAM_CARDDETECTOR_H
#define MAINPROGRAM_CARDDETECTOR_H

#include "common.hpp"
void analyzeCard(string const filename, bool& result, float& ratio, Mat&);
void drawExtendedLine(Mat &src, Point x1, Point x2);
vector<tuple<vector<Point>, vector<vector<Point>>>> get90AngleLineSets(vector<Vec4i> lines);
bool verify4PointsOfCard(vector<Point> points, float& ratio, vector<Point>&);
bool nailDownPoints(vector<tuple<Point, vector<tuple<Point, Point>>>>, int, int);
bool sortDistance(tuple<Point, Point> i, tuple<Point, Point> j);
#endif //MAINPROGRAM_CARDDETECTOR_H
