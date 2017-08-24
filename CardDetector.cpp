//
// Created by 김우현 on 28/07/2017.
//

#include "CardDetector.h"

void printVector(tuple<Point, vector<tuple<Point, Point>>> input);

void analyzeCard(string const filename, bool &result, float &ratio, Mat& resultImg) {
    Mat src = imread(filename, IMREAD_COLOR);
    if (!src.data) {
        printf("No image data \n");
        return;
    }
//    rotate(src,  90, src);
    rotate_90n(src, src, 0);
    resizeImage(src);
    blurImage(src);

    Mat greySrc;
    cvtColor(src, greySrc, CV_BGR2GRAY);
    Mat greySrcForCard = greySrc.rowRange(Range(0, greySrc.rows*0.339));

    Mat afterThreshold;
    threshold(greySrcForCard, afterThreshold, 160, 255, 0);
    Mat canny;
    Canny(afterThreshold, canny, 160, 180, 3);

    // hough lines
    Mat hough = src.clone();

    vector<Vec4i> lines;
    HoughLinesP(canny, lines, 1, CV_PI / 180, 90, 50, 30);

    vector<Vec4i> cleanedLines;

    deleteDuplicateLines(lines, cleanedLines);
#if(showGuideLines)
    for (size_t i = 0; i < cleanedLines.size(); i++) {
        putText(hough, to_string(i), Point(cleanedLines[i][0], cleanedLines[i][1]), FONT_HERSHEY_PLAIN, 1.0, 0, 2.0);
        line(hough, Point(cleanedLines[i][0], cleanedLines[i][1]), Point(cleanedLines[i][2], cleanedLines[i][3]),
             Scalar(0, 0, 255), 3, 8);
        drawExtendedLine(hough, Point(cleanedLines[i][0], cleanedLines[i][1]),
                         Point(cleanedLines[i][2], cleanedLines[i][3]));
    }
#endif
    // 직각에 가깝게 만나는 선들을 검색
    vector<tuple<vector<Point>, vector<vector<Point>>>> degreeMappedLines = get90AngleLineSets(cleanedLines);

    vector<Point> intersectsList;
    for (int j = 0; j < degreeMappedLines.size(); j++) {
        vector<Point> startingLine;
        vector<vector<Point>> correspondingLines;

        tie(startingLine, correspondingLines) = degreeMappedLines[j];
        for (int i = 0; i < correspondingLines.size(); i++) {
            vector<Point> correspondingLine = correspondingLines[i];
            Point intersect = getIntersect(startingLine[0], startingLine[1], correspondingLine[0],
                                           correspondingLine[1]);
            // 서로 수직인 직선들 조합 중 교점이 두 직선 위 어느 곳에도 있지 않을 경우(교점)를 추려냄
            if (!isPointOnTheLine(intersect, startingLine) && !isPointOnTheLine(intersect, correspondingLine)) {
                if (find(intersectsList.begin(), intersectsList.end(), intersect) == intersectsList.end()) {
                    // 없다면
                    intersectsList.push_back(intersect);
                }
            }
        }
    }

    for (vector<Point>::iterator it = intersectsList.begin(); it != intersectsList.end();) {
        if ((*it).x <= 0 || (*it).x >= src.cols || (*it).y <= 0 || (*it).y >= src.rows) {
            it = intersectsList.erase(it);
        } else {
#if(showGuideLines)
            circle(hough, Point((*it).x, (*it).y), 3, Scalar(255, 0, 0), 3, 8);
#endif
            it++;
        }
    }

    vector<Point> resultPointsSorted;

    vector<Point> target4Points;

    double const cardRatioNormal = 1.60377358490566;
    // 검색된 점의 개수가 4보다 작다면 카드 인식 실패임
    if (intersectsList.size() < 4) {
        // 검색된 점의 개수가 2일 경우 한면이 검색되지 않은 것인지 확인
        if(intersectsList.size() == 2){
            // 2개 점 검색되었다면 나머지 2개를 유추
            // TODO 추후 개선 필요 // 지금은 가정임
            // 두 점의 선이 가로인지 세로인지 확인
            Point pointA = intersectsList[0];
            Point pointB = intersectsList[1];

            if(abs(pointA.x - pointB.x) > abs(pointA.y - pointB.y)){
                // 가로 선
                float length = getDistance(pointA, pointB);
                float height = length/cardRatioNormal;
                // TODO 위에 선인지 아래선인지 판별 필요
                target4Points = {Point(pointA.x, cvFloor(pointA.y+height)), pointA, Point(pointB.x, cvFloor(pointB.y+height)), pointB};
            }else{
                // 세로 선
                float length = getDistance(pointA, pointB);
                bool isLeft = pointA.x < hough.cols;
                float width = length*cardRatioNormal;
                Point upper = pointA.y < pointB.y ? pointA : pointB;
                Point lower = pointA.y < pointB.y ? pointB : pointA;
                if(isLeft){
                    target4Points = {
                            lower,
                            upper,
                            Point(cvFloor(lower.x+width), lower.y),
                            Point(cvFloor(upper.x+width), upper.y),
                    };
                } else{
                    target4Points = {
                            lower,
                            upper,
                            Point(cvFloor(lower.x-width), lower.y),
                            Point(cvFloor(upper.x-width), upper.y),
                    };
                }
            }
            result = verify4PointsOfCard(target4Points, ratio, resultPointsSorted);
        }else if(intersectsList.size() == 3){
           result = false;
        }else{
            // size == 1
            result = false;
        }

    }
        // 검색된 점이 4개라면 타당성 검증으로 넘어가고 그 이상이라면 추려내기 작업 시작
    else if (intersectsList.size() > 4) {
        // 추려내기
        vector<tuple<Point, Point>> pointLineList;
        for (int i = 0; i < intersectsList.size(); i++) {
            for (int j = 0; j < intersectsList.size(); j++) {
                if (j <= i)continue;
                pointLineList.push_back(make_tuple(intersectsList[i], intersectsList[j]));
            }
        }
        sort(pointLineList.begin(), pointLineList.end(), sortDistance);
        long maxCount = pointLineList.size();
        for (int i = 0; i < maxCount; i++) {
            for (int j = 0; j < maxCount; j++) {
                if (j <= i)continue;
                Point a1 = get<0>(pointLineList[i]);
                Point a2 = get<1>(pointLineList[i]);
                Point b1 = get<0>(pointLineList[j]);
                Point b2 = get<1>(pointLineList[j]);
                // TODO 각도는 생략
                Point intersect = getIntersect(a1, a2, b1, b2);
                if (!isPointOnTheLine(intersect, {a1, a2}) || !isPointOnTheLine(intersect, {b1, b2})) {
                    continue;
                } else {
                    float distA = abs(getDistance(intersect, a1) - getDistance(intersect, a2));
                    float distB = abs(getDistance(intersect, b1) - getDistance(intersect, b2));
                    if (distA > 50 ||
                        distB > 50) {
                        continue;
                    } else {
                        if (abs(distA - distB) > 10) {
                            continue;
                        } else {
                            if (
                                    isPointOnTheLine(intersect, {a1, b1})
                                    || isPointOnTheLine(intersect, {a1, b2})
                                    || isPointOnTheLine(intersect, {a2, b1})
                                    || isPointOnTheLine(intersect, {a2, b2})) {
                                continue;
                            } else {
#if(showGuideLines)
                                circle(hough, intersect, 3, Scalar(255, 0, 0), 3, 8);
                                line(hough, a1, a2, Scalar(0, 255, 255), 3, 8);
                                line(hough, b1, b2, Scalar(0, 255, 255), 3, 8);
#endif
                                target4Points = {a1, a2, b1, b2};
                                goto stop;
                            }
                        }
                    }
                }
            }
        }

        stop:

        result = verify4PointsOfCard(target4Points, ratio, resultPointsSorted);
    } else {
// 검색된 점의 타당성 검증
        vector<Point> target4Points = {intersectsList[0], intersectsList[1],
                                       intersectsList[2], intersectsList[3]};
        result = verify4PointsOfCard(target4Points, ratio, resultPointsSorted);
    }

    if(result){
        // 카드 번호 가려주기
        Vec3b color =hough.at<Vec3b>(Point(resultPointsSorted[0].x-5, resultPointsSorted[0].y-5));
        const Point* resultPoints = resultPointsSorted.data();
        fillConvexPoly(hough, resultPoints, 4 ,Scalar(color[0], color[1], color[2]), 16, 0);
        resultImg = hough.rowRange(Range(cv::max(resultPointsSorted[0].y,resultPointsSorted[2].y)+20, hough.rows));
    }else{
        resultImg = hough.rowRange(Range(cvFloor(hough.rows*0.339), hough.rows));
    }

//    imshow("hough", hough);



}

vector<tuple<vector<Point>, vector<vector<Point>>>> get90AngleLineSets(vector<Vec4i> lines) {
    vector<tuple<vector<Point>, vector<vector<Point>>>> result;
    for (int i = 0; i < lines.size(); i++) {
        Point x1 = Point(lines[i][0], lines[i][1]);
        Point x2 = Point(lines[i][2], lines[i][3]);
        vector<Point> initLine = {x1, x2};
        vector<vector<Point>> resultLines;
        for (int j = 0; j < lines.size(); j++) {
            if (j <= i)continue;
            Point y1 = Point(lines[j][0], lines[j][1]);
            Point y2 = Point(lines[j][2], lines[j][3]);
            if (x1 == y1 && x2 == y2) {
                continue;
            }
            Point intersect = getIntersect(x1, x2, y1, y2);
            if (abs(acos(
                    -1 * (pow(getDistance(x1, y1), 2) - pow(getDistance(intersect, x1), 2) -
                          pow(getDistance(intersect, y1), 2))
                    / (2 * getDistance(intersect, x1) * getDistance(intersect, y1))
            )) < 10) {
                resultLines.push_back({y1, y2});
            }
        }
        if (resultLines.size() != 0)
            result.push_back(make_tuple(initLine, resultLines));
    }
    return result;
}

bool sortDistance(tuple<Point, Point> i, tuple<Point, Point> j) {
    return getDistance(get<0>(i), get<1>(i)) > getDistance(get<0>(j), get<1>(j));
}

bool sortHorizontal(Point i, Point j) {
    return i.x < j.x;
}

bool sortVertical(Point i, Point j) {
    return i.y > j.y;
}

bool verify4PointsOfCard(vector<Point> points, float &ratio, vector<Point>& pointsSorted) {
    assert(points.size() == 4);
    Point topLeft, topRight, bottomLeft, bottomRight;
    sort(points.begin(), points.end(), sortHorizontal);
    sort(points.begin(), points.end() - 2, sortVertical);
    sort(points.begin() + 2, points.end(), sortVertical);
    bottomLeft = points[0];
    topLeft = points[1];
    bottomRight = points[2];
    topRight = points[3];
    pointsSorted = {topLeft, bottomLeft, bottomRight, topRight};
    if (getDistance(bottomLeft, bottomRight) / getDistance(topLeft, topRight) < 0.8) {
        // compare horizontal length
        // 위 아래 가로길이 불일치
        ratio = 0;
        cerr << "위 아래 가로길이 불일치" << endl;
        return false;
    } else if (getDistance(bottomLeft, bottomRight) / getDistance(topLeft, topRight) < 0.8) {
        // compare vertical length
        // 왼쪽 오른쪽 세로 길이 불일치
        ratio = 0;
        cerr << "왼쪽 오른쪽 세로 길이 불일치" << endl;
        return false;
    } else {
        float width = getDistance(bottomLeft, bottomRight);
        ratio = 86 / width;
        return true;
    }
}