#ifndef INCLUDED_BEZIER_CURVE_EVALUATOR_H
#define INCLUDED_BEZIER_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  
#pragma warning(disable : 4018)
#pragma warning(disable : 4267)
#pragma warning(disable : 4311)
#pragma warning(disable : 4312)
#pragma warning(disable : 4244)
#pragma warning(disable : 4305)

#include "curveevaluator.h"

struct Bezier {
    Point v0;
    Point v1;
    Point v2;
    Point v3;
};

class BezierCurveEvaluator : public CurveEvaluator
{
protected:
    virtual const char* getCurveName() const;
    virtual unsigned int getControlPointStep() const;
    virtual Bezier getBezier(const Point& p0, const Point& p1, const Point& p2, const Point& p3) const;
    virtual const std::vector<Point> fillOutControlPoints(const std::vector<Point>& realControlPoints, const float xMax, const bool wrap) const;
    
private:
    bool isApproximatelyLinear(const Point& v0, const Point& v1, const Point& v2, const Point& v3) const;
    void evaluateBezierCurve(std::vector<Point>& evaluatedPoints, const Point& v0, const Point& v1, const Point& v2, const Point& v3, const float nextKnownX, const float xMax, const int depth) const;

public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
};

#endif
