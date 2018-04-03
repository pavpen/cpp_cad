#ifndef _CPP_CAD_ALGEBRA_VECTOR_3_H
#define _CPP_CAD_ALGEBRA_VECTOR_3_H

#include <tuple>

#include <CGAL/number_utils.h>


namespace cpp_cad
{

namespace vector_3
{

template <class PointType>
inline std::tuple<double, double, double> point_to_tuple(const PointType &p)
{
    return std::make_tuple(
        CGAL::to_double(p.x()),
        CGAL::to_double(p.y()),
        CGAL::to_double(p.z()) );
}

inline std::tuple<double, double, double> operator-(
    const std::tuple<double, double, double> &a)
{
    return std::make_tuple(
        std::get<0>(a),
        std::get<1>(a),
        std::get<2>(a) );
}

inline std::tuple<double, double, double> operator+(
    const std::tuple<double, double, double> &a,
    const std::tuple<double, double, double> &b)
{
    return std::make_tuple(
        std::get<0>(a) + std::get<0>(b),
        std::get<1>(a) + std::get<1>(b),
        std::get<2>(a) + std::get<2>(b) );
}

inline std::tuple<double, double, double> operator-(
    const std::tuple<double, double, double> &a,
    const std::tuple<double, double, double> &b)
{
    return std::make_tuple(
        std::get<0>(a) - std::get<0>(b),
        std::get<1>(a) - std::get<1>(b),
        std::get<2>(a) - std::get<2>(b) );
}

inline std::tuple<double, double, double> operator*(
    const std::tuple<double, double, double> &a,
    double b)
{
    return std::make_tuple(
        b * std::get<0>(a),
        b * std::get<1>(a),
        b * std::get<2>(a) );
}

inline double norm_3_squared(double x, double y, double z)
{
    return x*x + y*y + z*z;
}

inline std::tuple<double, double, double> cross_3(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    return std::make_tuple(
        y1 * z2 - z1 * y2,
        z1 * x2 - x1 * z2,
        x1 * y2 - y1 * x2
    );
}

double dot_3(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    return x1 * x2 + y1 * y2 + z1 * z2;
}

// Returns some unit vector orthogonal to a given vector.
inline std::tuple<double, double, double> find_perpendicular_axis(
    double x, double y, double z)
{
    if (fabs(x) <= fabs(y) && fabs(x) <= fabs(z))
    {
        // Use `res` = `input` cross `x`:
        return std::make_tuple(0, z, -y);
    }
    else if (fabs(y) <= fabs(x) && fabs(y) <= fabs(z))
    {
        // Use `res` = `input` cross `y`:
        return std::make_tuple(-z, 0, -y);
    }
    else
    {
        // Use `res` = `input` cross `z`:
        return std::make_tuple(y, -x, 0);
    }
}

// Returns the squared distance between two points.
template <class PointType>
inline double distance_3_squared(const PointType &p0, const PointType &p1)
{
    return norm_3_squared(
        p1.x() - p0.x(), p1.y() - p0.y(), p1.z() - p0.z());
}

}

}

#endif // _CPP_CAD_ALGEBRA_VECTOR_3_H
