#include "catmullromcurveevaluator.h"
#include <cassert>

const char* CatmullRomCurveEvaluator::getCurveName() const
{
    return "Catmull-Rom";
}

unsigned int CatmullRomCurveEvaluator::getControlPointStep() const
{
    return 1;
}

const std::vector<Point> CatmullRomCurveEvaluator::fillOutControlPoints(const std::vector<Point>& realControlPoints, const float xMax, const bool wrap) const
{
    std::vector<Point> points;
    
    const size_t controlPointCount = realControlPoints.size();
    
    if (wrap)
    {
        points.push_back(Point(realControlPoints[controlPointCount - 2].x - xMax, realControlPoints[controlPointCount - 2].y));
        points.push_back(Point(realControlPoints[controlPointCount - 1].x - xMax, realControlPoints[controlPointCount - 1].y));
    }
    else
    {
        const Point firstPoint(0, realControlPoints[0].y);
        points.push_back(firstPoint);
        points.push_back(firstPoint);
    }
    
    points.insert(points.cend(), realControlPoints.cbegin(), realControlPoints.cend());
    
    if (wrap)
    {
        points.push_back(Point(xMax + realControlPoints[0].x, realControlPoints[0].y));
        points.push_back(Point(xMax + realControlPoints[1].x, realControlPoints[1].y));
    }
    else
    {
        const Point lastPoint(xMax, realControlPoints[controlPointCount - 1].y);
        points.push_back(lastPoint);
        points.push_back(lastPoint);
    }

    return points;
}

Bezier CatmullRomCurveEvaluator::getBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const
{
    Bezier curve;
    curve.v0 = p1;
    curve.v1 = p1 + (p2 - p0) / 6.0f;
    curve.v2 = p2 - (p3 - p1) / 6.0f;
    curve.v3 = p2;
    
    return curve;
}
