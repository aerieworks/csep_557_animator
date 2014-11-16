#include "bsplinecurveevaluator.h"
#include <cassert>
#include <math.h>
#include <iostream>

using namespace std;

const char* BSplineCurveEvaluator::getCurveName() const
{
    return "B-Spline";
}

Bezier BSplineCurveEvaluator::getBezier(const std::vector<Point>& controlPoints, const size_t index, bool& hasMore) const
{
    const size_t lastIndex = controlPoints.size() - 1;
    const Point b0 = controlPoints[fmax(0ul, index - 2)];
    const Point b1 = controlPoints[fmax(0ul, index - 1)];
    const Point b2 = controlPoints[fmin(lastIndex, index)];
    const Point b3 = controlPoints[fmin(lastIndex, index + 1)];
    
    cerr << "  de Boor points: "
        << "(" << b0.x << ", " << b0.y << "), "
        << "(" << b1.x << ", " << b1.y << "), "
        << "(" << b2.x << ", " << b2.y << "), "
        << "(" << b3.x << ", " << b3.y << ")" << endl;
    
    Bezier curve;
    curve.v0 = (b0 + b2) * (1.0 / 6.0) + b1 * (2.0 / 3.0);
    curve.v1 = (b1 * 2 + b2) / 3.0;
    curve.v2 = (b1 + b2 * 2) / 3.0;
    curve.v3 = (b1 + b3) * (1.0 / 6.0) + b2 * (2.0 / 3.0);
    
    hasMore = (index < controlPoints.size());
    return curve;
}
