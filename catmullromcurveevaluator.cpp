#include "catmullromcurveevaluator.h"
#include <cassert>
#include <iostream>

using namespace std;

void CatmullRomCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
    const size_t controlPointCount = ptvCtrlPts.size();
    std::vector<Point> bezierPoints;//((controlPointCount - 1) * 3 + 1);

    // Duplicate first control point to ensure interpoliation.
    bezierPoints.push_back(Point(ptvCtrlPts[0]));
    
    cerr << "Bounds: [0, " << (controlPointCount - 1) << ")" << endl;
    for (size_t i = 0; i < controlPointCount - 1; i++)
    {
        const size_t p0Index = (i == 0 ? 0 : (i - 1));
        const size_t p3Index = min(controlPointCount - 1, i + 2);
        const Point p0 = bezierPoints[p0Index];
        const Point p1 = ptvCtrlPts[i];
        const Point p2 = ptvCtrlPts[i + 1];
        const Point p3 = ptvCtrlPts[p3Index];

        cerr << "[" << i << "] Using: ..., "
            << p0Index << ":(" << p0.x << ", " << p0.y << "), "
            << i << ":(" << p1.x << ", " << p1.y << "), "
            << (i + 1) << ":(" << p2.x << ", " << p2.y << "), "
            << p3Index << ":(" << p3.x << ", " << p3.y << ")" << endl;
        /*bezierPoints[(i * 3) + 1] = p1 + (p2 - p0) / 6.0f;
        bezierPoints[(i * 3) + 2] = p2 - (p3 - p1) / 6.0f;
        bezierPoints[(i * 3) + 3] = p2;*/
        bezierPoints.push_back(p1 + (p2 - p0) / 6.0f);
        bezierPoints.push_back(p2 - (p3 - p1) / 6.0f);
        bezierPoints.push_back(Point(p2));
    }
    
    BezierCurveEvaluator::evaluateCurve(bezierPoints, ptvEvaluatedCurvePts, fAniLength, bWrap);
}
