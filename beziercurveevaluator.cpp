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
    
    cerr << "Evaluating " << getCurveName() << " from " << ptvCtrlPts.size() << " control points: " << endl;
    for (std::vector<Point>::const_iterator it = ptvCtrlPts.cbegin(); it != ptvCtrlPts.cend(); ++it)
    {
        cerr << "  (" << (*it).x << ", " << (*it).y << ")" << endl;
    }

    const std::vector<Point> controlPoints = fillOutControlPoints(ptvCtrlPts, fAniLength, bWrap);
    cerr << "Filled out control points (" << controlPoints.size() << "):" << endl;
    for (std::vector<Point>::const_iterator it = controlPoints.cbegin(); it != controlPoints.cend(); ++it)
    {
        cerr << "  (" << (*it).x << ", " << (*it).y << ")" << endl;
    }
    
    const unsigned int pointStep = getControlPointStep();
    const size_t controlPointCount = controlPoints.size();
    size_t previousPointCount = 0;
    size_t pointIndex = 0;
    size_t curveCount = 0;
    while (pointIndex + 3 < controlPointCount)
    {
        curveCount += 1;
        cerr << "Curve " << curveCount << ":" << endl;
        
        // Get the next Bezier curve points from the control points.
        const Point& p0 = controlPoints[pointIndex];
        const Point& p1 = controlPoints[pointIndex + 1];
        const Point& p2 = controlPoints[pointIndex + 2];
        const Point& p3 = controlPoints[pointIndex + 3];
        cerr << "  Control points: "
            << "(" << p0.x << ", " << p0.y << "), "
            << "(" << p1.x << ", " << p1.y << "), "
            << "(" << p2.x << ", " << p2.y << "), "
            << "(" << p3.x << ", " << p3.y << ")" << endl;
        
        Bezier curve = getBezier(p0, p1, p2, p3);
        cerr << "  Bezier points: "
            << "(" << curve.v0.x << ", " << curve.v0.y << "), "
            << "(" << curve.v1.x << ", " << curve.v1.y << "), "
            << "(" << curve.v2.x << ", " << curve.v2.y << "), "
            << "(" << curve.v3.x << ", " << curve.v3.y << ")" << endl;
        
        evaluateBezierCurve(ptvEvaluatedCurvePts, curve.v0, curve.v1, curve.v2, curve.v3, curve.v3.x, fAniLength, 0);
        cerr << "  Generated " << (ptvEvaluatedCurvePts.size() - previousPointCount) << " points." << endl;
        previousPointCount = ptvEvaluatedCurvePts.size();
        pointIndex += pointStep;
    }
    
    const size_t pointCount = ptvEvaluatedCurvePts.size();
    cerr << "Generated " << pointCount << " points total in " << curveCount << " curves." << endl;
    cerr << "First points: ";
    for (int i = 0; i < 3; i++)
    {
        cerr << " (" << ptvEvaluatedCurvePts[i].x << ", " << ptvEvaluatedCurvePts[i].y << ") ";
    }
    cerr << endl << "Last points: ";
    for (int i = 1; i <= 3; i++)
    {
        cerr << " (" << ptvEvaluatedCurvePts[pointCount - i].x << ", " << ptvEvaluatedCurvePts[pointCount - i].y << ") ";
    }
    cerr << endl;
}

bool BezierCurveEvaluator::isApproximatelyLinear(const Point& v0, const Point& v1, const Point& v2, const Point& v3) const
{
    const float actualDistance = fabs(v0.distance(v1)) + fabs(v1.distance(v2)) + fabs(v2.distance(v3));
    const float linearDistance = fabs(v0.distance(v3));
    return (linearDistance == 0.0) || (actualDistance / linearDistance < 1.0 + BEZIER_LINEAR_EPSILON);
}

void BezierCurveEvaluator::evaluateBezierCurve(std::vector<Point>& evaluatedPoints, const Point& v0, const Point& v1, const Point& v2, const Point& v3, const float nextKnownX, const float xMax, const int depth) const
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
        const float m = (v3.y - v0.y) / (v3.x - v0.x);
        const float b = v0.y - m * v0.x;
        if ((evaluatedPoints.size() == 0 || v0.x >= evaluatedPoints[evaluatedPoints.size() - 1].x)
            && v0.x <= nextKnownX && v0.x <= xMax)
        {
            if (v0.x < 0)
            {
                if (v3.x > 0)
                {
                    // If this line segment would cross the Y axis, add the Y-axis intercept point instead of v0.
                    // Solve y = mx + b for x = 0, given v0 and v3.
                    evaluatedPoints.push_back(Point(0, b));
                }
            }
            else
            {
                evaluatedPoints.push_back(Point(v0));
            }
        }
        if ((evaluatedPoints.size() == 0 || v3.x >= evaluatedPoints[evaluatedPoints.size() - 1].x)
            && v3.x >= 0 && v3.x <= nextKnownX)
        {
            if (v3.x > xMax)
            {
                if (v0.x < xMax)
                {
                    // If this line segment would cross xMax, add the xMax intercept point instead of v3.
                    // Solve y = mx + b for x = xMax, given v0 and v3.
                    const float m = (v3.y - v0.y) / (v3.x - v0.x);
                    const float b = v0.y - m * v0.x;
                    evaluatedPoints.push_back(Point(xMax, m * xMax + b));
                }
            }
            else
            {
                evaluatedPoints.push_back(Point(v3));
            }
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
        
        evaluateBezierCurve(evaluatedPoints, l0, l1, l2, l3, nextKnownX, xMax, depth + 1);
        evaluateBezierCurve(evaluatedPoints, r0, r1, r2, r3, nextKnownX, xMax, depth + 1);
    }
}

const char* BezierCurveEvaluator::getCurveName() const
{
    return "Bezier";
}

unsigned int BezierCurveEvaluator::getControlPointStep() const
{
    return 3;
}

const std::vector<Point> BezierCurveEvaluator::fillOutControlPoints(const std::vector<Point>& realControlPoints, const float xMax, const bool wrap) const
{
    const size_t realPointCount = realControlPoints.size();
    
    std::vector<Point> points;
    const Point& firstPoint = realControlPoints[0];
    const Point& secondPoint = realControlPoints[1];
    const Point& secondToLastPoint = realControlPoints[realPointCount - 2];
    const Point& lastPoint = realControlPoints[realPointCount - 1];
    if (wrap)
    {
        points.push_back(Point(lastPoint.x - xMax, lastPoint.y));
        points.push_back(Point(lastPoint.x * 2.0 - secondToLastPoint.x - xMax, lastPoint.y * 2.0 - secondToLastPoint.y));
        points.push_back(Point(firstPoint.x * 2.0 - secondPoint.x, firstPoint.y * 2.0 - secondPoint.y));
    }
    else
    {
        points.push_back(Point(0, firstPoint.y));
        points.push_back(Point(1.0 / 3.0, firstPoint.y));
        points.push_back(Point(firstPoint.x - (1.0 / 3.0), firstPoint.y));
    }

    points.insert(points.cend(), realControlPoints.cbegin(), realControlPoints.cend());
    for (size_t i = (realPointCount - 1) % 3; i > 0 && i < 3; i++)
    {
        points.push_back(Point(lastPoint));
    }

    if (wrap)
    {
        points.push_back(Point(lastPoint.x * 2.0 - secondToLastPoint.x, lastPoint.y * 2.0 - secondToLastPoint.y));
        points.push_back(Point(xMax + firstPoint.x * 2.0 - secondPoint.x, firstPoint.y * 2.0 - secondPoint.y));
        points.push_back(Point(xMax + firstPoint.x, firstPoint.y));
    }
    else
    {
        points.push_back(Point(lastPoint.x + (1.0 / 3.0), lastPoint.y));
        points.push_back(Point(xMax - (1.0 / 3.0), lastPoint.y));
        points.push_back(Point(xMax, lastPoint.y));
        
    }
    return points;
}

Bezier BezierCurveEvaluator::getBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const
{
    Bezier curve;
    curve.v0 = p0;
    curve.v1 = p1;
    curve.v2 = p2;
    curve.v3 = p3;
    
    return curve;
}
