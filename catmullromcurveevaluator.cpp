#include "catmullromcurveevaluator.h"
#include <cassert>

const char* CatmullRomCurveEvaluator::getCurveName() const
{
    return "Catmull-Rom";
}

Bezier CatmullRomCurveEvaluator::getBezier(const std::vector<Point>& controlPoints, const size_t index, bool& hasMore) const
{
    const size_t lastIndex = controlPoints.size() - 1;
    const Point p0 = (index == 0 ? controlPoints[0] : controlPoints[index - 1]);
    const Point p1 = controlPoints[index];
    const Point p2 = controlPoints[index + 1];
    const Point p3 = controlPoints[min(lastIndex, index + 2)];
    
    Bezier curve;
    curve.v0 = p1;
    curve.v1 = p1 + (p2 - p0) / 6.0f;
    curve.v2 = p2 - (p3 - p1) / 6.0f;
    curve.v3 = p2;
    
    hasMore = (index < lastIndex - 1);
    return curve;
}
