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
    virtual Bezier getBezier(const std::vector<Point>& controlPoints, const size_t bezierIndex, bool& hasMore) const;
};

#endif
