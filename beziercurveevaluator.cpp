#include "beziercurveevaluator.h"
#include <cassert>
#include <iostream>
#include <math.h>

using namespace std;

#define BEZIER_LINEAR_EPSILON (1e-4)
#define MAX_EVALUATE_RECURSION_DEPTH 1000

void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
    ptvEvaluatedCurvePts.clear();
    
	const size_t controlPointCount = ptvCtrlPts.size();
    cerr << "Evaluating " << getCurveName() << " from " << controlPointCount << " control points: " << endl;
    for (std::vector<Point>::const_iterator it = ptvCtrlPts.cbegin(); it != ptvCtrlPts.cend(); ++it)
    {
        cerr << "  (" << (*it).x << ", " << (*it).y << ")" << endl;
    }

    // Find v0 for the first curve and v3 for the last bezier curve.
    const Point firstV0 = ptvCtrlPts[0];
    const Point lastV3 = ptvCtrlPts[controlPointCount - 1];
    
    float wrapLineSlope;
    float wrapYValue;
    if (bWrap)
    {
        // If wrapping is enabled then we want to fill in any gaps between t=0 and first v0 and last v3 and t=END with segments of the line that would connect
        // last v3 to first v0 if the timeline wrapped from t=END to t=0.
        // Compute the slope of that line, and the correct Y value for the wrapping point (i.e. both time=0 and time=END).
        wrapLineSlope = (firstV0.y - lastV3.y) / (firstV0.x + (fAniLength - lastV3.x));
        wrapYValue = firstV0.y - wrapLineSlope * firstV0.x;
    }
    
    if (firstV0.x > 0.0)
    {
        // The first Bezier doesn't start at time=0, so let's fill in that space with something sensible.
        // If wrapping is enabled, we'll connect first v0 to the Y value given by the wrapping line function.
        // If not, we'll just reuse first v0's Y value.
        ptvEvaluatedCurvePts.push_back(Point(0.0, bWrap ? wrapYValue : firstV0.y));
        cerr << "Generated point to extend curve to full animation length." << endl;
    }

    bool hasMore;
    size_t bezierIndex = 0;
    size_t pointCount = ptvEvaluatedCurvePts.size();
    do
    {
        // Get the next Bezier curve from the control points.
        Bezier curve = getBezier(ptvCtrlPts, bezierIndex, hasMore);
        cerr << "Curve " << bezierIndex << ": " << endl
            << "  (" << curve.v0.x << ", " << curve.v0.y << "), "
            << "(" << curve.v1.x << ", " << curve.v1.y << "), "
            << "(" << curve.v2.x << ", " << curve.v2.y << "), "
            << "(" << curve.v3.x << ", " << curve.v3.y << ")" << endl;
        evaluateBezierCurve(ptvEvaluatedCurvePts, curve.v0, curve.v1, curve.v2, curve.v3, curve.v3.x, 0);
        cerr << "  Generated " << (ptvEvaluatedCurvePts.size() - pointCount) << " points." << endl;
        pointCount += ptvEvaluatedCurvePts.size();
        bezierIndex += 1;
    } while (hasMore);
    
    if (lastV3.x < fAniLength)
    {
        // The last Bezier doesn't end at time=END, so let's fill in that space with something sensible.
        // If wrapping is enabled, we'll connect last v3 to the Y value given by the wrapping line function.
        // If not, we'll just reuse last v3's Y value.
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, bWrap ? wrapYValue : lastV3.y));
        cerr << "Generated point to extend curve to full animation length." << endl;
    }
    
    cerr << "Generated " << ptvEvaluatedCurvePts.size() << " points total." << endl;
}

bool BezierCurveEvaluator::isApproximatelyLinear(const Point& v0, const Point& v1, const Point& v2, const Point& v3) const
{
    const float actualDistance = fabs(v0.distance(v1)) + fabs(v1.distance(v2)) + fabs(v2.distance(v3));
    const float linearDistance = fabs(v0.distance(v3));
    return (linearDistance == 0.0) || (actualDistance / linearDistance < 1.0 + BEZIER_LINEAR_EPSILON);
}

void BezierCurveEvaluator::evaluateBezierCurve(std::vector<Point>& evaluatedPoints, const Point& v0, const Point& v1, const Point& v2, const Point& v3, const float maxX, const int depth) const
{
    if (depth >= MAX_EVALUATE_RECURSION_DEPTH || isApproximatelyLinear(v0, v1, v2, v3))
    {
        // This section of the curve is linear enough (or we've iterated enough) that we are just going to render it as
        // a line from v0 to v3.
        
        // We want to maintain monotonicity along the X axis.  Only include each point if:
        // 1) The point comes after the last point we added, on the X axis.
        // 2) The point DOES NOT come after "maxX" (which is the X for the last control point).
        //      - This ensures that we can always include the last control point on the line, ensuring interpolation.
        //      - Catmull-Rom gets really funky around edge cases if you don't handle this case in some way.
        if (v0.x >= evaluatedPoints[evaluatedPoints.size() - 1].x && v0.x <= maxX)
        {
            evaluatedPoints.push_back(Point(v0));
        }
        if (v3.x >= evaluatedPoints[evaluatedPoints.size() - 1].x && v3.x <= maxX)
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
        
        evaluateBezierCurve(evaluatedPoints, l0, l1, l2, l3, maxX, depth + 1);
        evaluateBezierCurve(evaluatedPoints, r0, r1, r2, r3, maxX, depth + 1);
    }
}

const char* BezierCurveEvaluator::getCurveName() const
{
    return "Bezier";
}

Bezier BezierCurveEvaluator::getBezier(const std::vector<Point>& controlPoints, const size_t index, bool& hasMore) const
{
    /*
     Bezier curves have four control points.  We want to reuse v3 from the previous curve as v0 of the next curve.
     We also fill out the last curve by reusing the last control point if necessary.
     Given 8 control points [p0, ... , p7], we want:
     - Curve 1: [p0, p1, p2, p3]
     - Curve 2: [p3, p4, p5, p6]
     - Curve 3: [p6, p7, p7, p7]
     */
    const size_t start = index * 3;
    const size_t lastIndex = controlPoints.size() - 1;
    
    Bezier curve;
    curve.v0 = controlPoints[start];
    curve.v1 = controlPoints[min(lastIndex, start + 1)];
    curve.v2 = controlPoints[min(lastIndex, start + 2)];
    curve.v3 = controlPoints[min(lastIndex, start + 3)];
    
    hasMore = (start < lastIndex);
    return curve;
}
