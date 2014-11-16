#include "bsplinecurveevaluator.h"
#include <cassert>
#include <math.h>
#include <iostream>

using namespace std;

const char* BSplineCurveEvaluator::getCurveName() const
{
    return "B-Spline";
}

unsigned int BSplineCurveEvaluator::getControlPointStep() const
{
    return 1;
}

const std::vector<Point> BSplineCurveEvaluator::fillOutControlPoints(const std::vector<Point>& realControlPoints, const float xMax, const bool wrap) const
{
    std::vector<Point> points;
    const size_t controlPointCount = realControlPoints.size();
    
    if (wrap)
    {
        for (size_t i = controlPointCount - 3; i < controlPointCount; i++)
        {
            const Point& point = realControlPoints[fmax(0, i)];
            points.push_back(Point(point.x - xMax, point.y));
        }
    }
    else
    {
        const Point firstPoint(0, realControlPoints[0].y);
        points.push_back(firstPoint);
        points.push_back(firstPoint);
        points.push_back(firstPoint);
    }
    
    points.insert(points.cend(), realControlPoints.cbegin(), realControlPoints.cend());
    
    if (wrap)
    {
        for (size_t i = 0; i < 3; i++)
        {
            const Point& point = realControlPoints[fmin(i, controlPointCount - 1)];
            points.push_back(Point(xMax + point.x, point.y));
        }
    }
    else
    {
        const Point lastPoint(xMax, realControlPoints[controlPointCount - 1].y);
        points.push_back(lastPoint);
        points.push_back(lastPoint);
        points.push_back(lastPoint);
    }
    
    return points;
}

Bezier BSplineCurveEvaluator::getBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const
{
    Bezier curve;
    curve.v0 = (p0 + p2) * (1.0 / 6.0) + p1 * (2.0 / 3.0);
    curve.v1 = (p1 * 2 + p2) / 3.0;
    curve.v2 = (p1 + p2 * 2) / 3.0;
    curve.v3 = (p1 + p3) * (1.0 / 6.0) + p2 * (2.0 / 3.0);
    
    return curve;
}
