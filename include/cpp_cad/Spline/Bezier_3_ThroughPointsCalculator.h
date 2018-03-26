#ifndef _CPP_CAD_BEZIER_3_THROUGH_POINTS_CALCULATOR_H
#define _CPP_CAD_BEZIER_3_THROUGH_POINTS_CALCULATOR_H

#include <iterator>
#include <vector>

#include "../reference_frame.h"
#include "../Spline/CubicBezierSegment.h"


namespace cpp_cad
{

// Calculates the control points for an open cubic Bézier spline through a
// sequence of given 3D points.
//
// Based on <https://www.particleincell.com/2012/bezier-splines/> by
// Lubos Brieda.
template <class Coordinate, class Point, class PointIterator>
class Bezier_3_ThroughPointsCalculator
{
    private:

    // Number of points to construct a spline through:
    int point_count;

    // Number of dimensions for each point:
    const int dimension_count = 3;

    // Points to construct a spline through:
    PointIterator &points_begin;
    const PointIterator &points_end;

    // Coefficients of the tri-diagonal matrix equation to solve.
    //     i = row number.
    //     a[i]: entries below the diagonal.
    //     b[i]: diagonal entries.
    //     c[i]: entries above the diagonal.
    //
    // We store the entries for all dimensions in each array.
    //     a[dimension_count * i + d]: Entry for row i, dimension d.
    //     b[dimension_count * i + d]: Entry for row i, dimension d.
    //     c[dimension_count * i + d]: Entry for row i, dimension d.
    std::vector<Coordinate> a;
    std::vector<Coordinate> b;
    std::vector<Coordinate> c;

    // Right-hand side of the matrix equation to solve.  Same format as
    // a, b, c above.
    std::vector<Coordinate> r;

    // Control point coordinates:
    std::vector<Coordinate> p1;
    std::vector<Coordinate> p2;

    // Spline segments. (Output.)
    std::vector<CubicBezierSegment<Point>> spline_segments;

    public:

    Bezier_3_ThroughPointsCalculator(
        PointIterator points_begin, PointIterator points_end)
    : point_count(std::distance(points_begin, points_end) - 1),
        points_begin(points_begin),
        points_end(points_end)
    {
        a.reserve(point_count * dimension_count);
        b.reserve(point_count * dimension_count);
        c.reserve(point_count * dimension_count);
        r.reserve(point_count * dimension_count);
        p1.resize(point_count * dimension_count);
        p2.resize(point_count * dimension_count);
        spline_segments.reserve(point_count);
    }

    void run()
    {
        // Calculate the control points:
        construct_eq();
        solve_eq();

        // Transfer the solution to the output splines:

        PointIterator &point_iter = points_begin;

        int end_i = point_count * dimension_count;
        for (int i = 0; i < end_i; i += dimension_count)
        {
            Point& point = *point_iter;

            ++point_iter;

            Point& next_point = *point_iter;

            spline_segments.emplace_back(
                point,
                Point(p1[i], p1[i + 1], p1[i + 2]),
                Point(p1[i], p1[i + 1], p2[i + 2]),
                next_point);
        }
    }

    private:

    // Constructs the tri-diagonal matrix and right-hand entries for the
    // equation to solve.
    void construct_eq()
    {
        PointIterator &point_iter = points_begin;

        // First segment:
        a.resize(d, 0); // Set `a` to `d` 0s.
        b.resize(d, 2); // Set to `d` 2s.
        c.resize(d, 1); // Set to `d` 1s.

        Point &point = *point_iter;

        ++point_iter;

        Point &next_point = *point_iter;

        for (int d = 0; d < dimension_count; ++d)
        {
            r.emplace_back(point[d] + next_point[d]);
        }

        // Internal segments:
        int i_end = point_count - 1;

        for (int i = 1; i < i_end; ++i)
        {
            a.resize(a.size() + d, 1); // Append `d` 1s.
            b.resize(b.size() + d, 4); // Append `d` 4s.
            c.resize(c.size() + d, 1); // Append `d` 1s.

            point = next_point;
            ++point_iter;
            next_point = *point_iter;

            for (int d = 0; d < dimension_count; ++d)
            {
                r.emplace_back(4 * point[d] + 2 * next_point[d]);
            }
        }

        // Last segment:
        a.resize(a.size() + d, 2); // Append `d` 2s.
        b.resize(b.size() + d, 7); // Append `d` 7s.
        c.resize(c.size() + d, 0); // Append `d` 0s.

        point = next_point;
        ++point_iter;
        next_point = *point_iter;

        for (int d = 0; d < dimension_count; ++d)
        {
            r.emplace_back(8 * point[d] + next_point[d]);
        }
    }

    // Solves the Ax = r matrix equation using the Thomas algorithm
    // (https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm).
    void solve_eq()
    {
        int end_i = point_count * dimension_count;
        for (int i = 1 * dimension_count; i < end_i; ++i)
        {
            Coordinate m = a[i] / b[i - 1];
            b[i] -= m * c[i - 1];
            r[i] -= m * r[i - 1];
        }

        // Compute the coordinate of the first control point p1 for each
        // Bezier segment.

        // p1[n-1] = r[n-1]/b[n-1] for all dimensions:
        end_i = point_count * dimension_count;
        for (int i = (point_count - 1) * dimension_count; i < end_i; ++i)
        {
            p1[i] = r[i] / b[i];
        }

        // p1[i] = (r[i] - c[i] * p1[i+1]) / b[i] for all dimensions:
        for (int i = (point_count - 2) * dimension_count; i >= 0; --i)
        {
            p1[i] = (r[i] - c[i] * p1[i + dimension_count]) / b[i];
        }

        // Now we have p1, compute p2:

        // p2[i] = 2 * K[i + 1] - p1[i + 1] for all dimensions:
        PointIterator &point_iter = points_begin;

        ++point_iter;
        i_end = (point_count - 1) * dimension_count;
        for (int i = 0, d = 0; i < i_end; ++i)
        {
            p2[i] = 2 * (*point_iter)[d] - p1[i + dimension_count];

            ++d;
            if (d >= dimension_count)
            {
                d = 0;
                ++point_iter;
            }
        }

        // p2[n - 1] = (K[n] + p1[n - 1]) / 2 for all dimensions:
        end_i = point_count * dimension_count;
        for (int i = (point_count - 1) * dimension_count, d = 0; i < end_i; ++i, ++d)
        {
            p2[i] = ( (*point_iter)[d] + p1[i] ) * 0.5;
        }
    }
};

}

#endif // _CPP_CAD_BEZIER_3_THROUGH_POINTS_CALCULATOR_H
