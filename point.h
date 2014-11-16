#ifndef POINT_H_INCLUDED
#define POINT_H_INCLUDED

#pragma warning(disable : 4786)

#include <functional>
#include <iostream>
#include <cmath>

class Point
{
public:

	Point(void);
	Point(const float& x, const float& y);

	void toStream(std::ostream& output_stream) const;
	void fromStream(std::istream& input_stream);
    
    friend Point operator+(const Point& lhs, const Point& rhs);
    friend Point operator-(const Point& lhs, const Point& rhs);
    friend Point operator*(const Point& lhs, const float rhs);
    friend Point operator/(const Point& lhs, const float rhs);

    float distance(const Point& p) const {
		float xd = x - p.x;
		float yd = y - p.y;
		return sqrtf(xd * xd + yd * yd);
	}
    
    Point midpoint(const Point &p) const {
        const float midx = (x + p.x) / 2.0;
        const float midy = (y + p.y) / 2.0;
        return Point(midx, midy);
    }

	float x;
	float y;
};

std::ostream& operator<<(std::ostream& output_stream, const Point& point);
std::istream& operator>>(std::istream& input_stream, Point& point);

class PointSmallerXCompare : public std::binary_function<const Point&, const Point&, bool>
{
public:
	bool operator()(const Point& first, const Point& second) const;
};

class PointLargerXCompare : public std::binary_function<const Point&, const Point&, bool>
{
public:
	bool operator()(const Point& first, const Point& second) const;
};

#endif // POINT_H_INCLUDED
