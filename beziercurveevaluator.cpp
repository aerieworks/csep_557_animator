#include "beziercurveevaluator.h"
#include <cassert>
#include <iostream>

using namespace std;

#define BEZIER_LINEAR_EPSILON (1e-5)

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
    ptvEvaluatedCurvePts.clear();
    
	const size_t controlPointCount = ptvCtrlPts.size();
    cerr << "Control points (" << controlPointCount << "): " << endl;
    for (std::vector<Point>::const_iterator it = ptvCtrlPts.cbegin(); it != ptvCtrlPts.cend(); ++it)
    {
        cerr << "  (" << (*it).x << ", " << (*it).y << ")" << endl;
    }
    
    const Point curveStartPoint = ptvCtrlPts[0];
    if (curveStartPoint.x > 0.0)
    {
        // The Bezier doesn't start at time = 0, so let's just add a horizontal segment from time = 0 to the start point.
        // This is C0 continuous, but it's not pretty; we could replace it with a separate bezier later.
        ptvEvaluatedCurvePts.push_back(Point(0.0, curveStartPoint.y));
    }
    
    // Iterate over the control points, constructing bezier curves along the way.
    // Keep going until we do not have enough points left to construct another curve.
    int bezierStart = 0;
    while (bezierStart + 4 <= controlPointCount)
    {
        // Evaluate the bezier curve formed by the current four control points.
        const Point v0 = ptvCtrlPts[bezierStart];
        const Point v1 = ptvCtrlPts[bezierStart + 1];
        const Point v2 = ptvCtrlPts[bezierStart + 2];
        const Point v3 = ptvCtrlPts[bezierStart + 3];
        evaluateBezierCurve(ptvEvaluatedCurvePts, v0, v1, v2, v3);
        
        // Beziers require 4 points, but we want to reuse v3 from the previous curve as v0 for the next curve.
        // This ensures C0 continuity.
        bezierStart += 3;
    }
    
    const Point lastCurveEnd = ptvCtrlPts[bezierStart];
    cerr << "Last curve end: " << bezierStart << " (" << lastCurveEnd.x << ", " << lastCurveEnd.y << ")" << endl;
    if (lastCurveEnd.x < fAniLength)
    {
        // The Bezier doesn't start at time = END, so let's just add a horizontal segment from the end of the curve to t= END.
        // This is C0 continuous, but it's not pretty; we could replace it with a separate bezier later.
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, lastCurveEnd.y));
    }
}

bool BezierCurveEvaluator::isApproximatelyLinear(const Point& v0, const Point& v1, const Point& v2, const Point& v3) const
{
    const float actualDistance = fabs(v0.distance(v1)) + fabs(v1.distance(v2)) + fabs(v2.distance(v3));
    const float linearDistance = fabs(v0.distance(v3));
    return (linearDistance == 0.0) || (actualDistance / linearDistance < 1.0 + BEZIER_LINEAR_EPSILON);
}

void BezierCurveEvaluator::evaluateBezierCurve(std::vector<Point>& evaluatedPoints, const Point& v0, const Point& v1, const Point& v2, const Point& v3) const
{
    if (isApproximatelyLinear(v0, v1, v2, v3))
    {
        const Point lastPoint = evaluatedPoints[evaluatedPoints.size() - 1];
        if (v0.x >= lastPoint.x)
        {
            evaluatedPoints.push_back(Point(v0));
        }
        if (v3.x >= v0.x && v3.x >= lastPoint.x)
        {
            evaluatedPoints.push_back(Point(v3));
        }
    }
    else
    {
        Point apex, l0 = v0, l1, l2, l3, r0, r1, r2, r3 = v3;
        l1 = v0.midpoint(v1);
        apex = v1.midpoint(v2);
        r2 = v2.midpoint(v3);

        l2 = l1.midpoint(apex);
        r1 = apex.midpoint(r2);
        l3 = r0 = l2.midpoint(r1);
        
        evaluateBezierCurve(evaluatedPoints, l0, l1, l2, l3);
        evaluateBezierCurve(evaluatedPoints, r0, r1, r2, r3);
    }
}
