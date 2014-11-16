#ifndef INCLUDED_B_SPLINE_CURVE_EVALUATOR_H
#define INCLUDED_B_SPLINE_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  
#pragma warning(disable : 4018)
#pragma warning(disable : 4267)
#pragma warning(disable : 4311)
#pragma warning(disable : 4312)
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)

#include "beziercurveevaluator.h"

class BSplineCurveEvaluator : public BezierCurveEvaluator
{
protected:
    virtual const char* getCurveName() const;
    virtual unsigned int getControlPointStep() const;
    virtual Bezier getBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const;
    virtual const std::vector<Point> fillOutControlPoints(const std::vector<Point>& realControlPoints, const float xMax, const bool wrap) const;
};

#endif
