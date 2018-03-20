#ifndef _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP
#define _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP

#include <operation_log.h>

#include "Polygon_2.h"


namespace cpp_cad
{

// Constructs an iterator over the series of polygons given by a polygon and a
// transform iterator.
template <class TransformIterator>
class Polygon_2_TransformsIterator
{
private:
    const Polygon_2 &polygon;
    TransformIterator transform_iterator;

public:
    Polygon_2_TransformsIterator(
        TransformIterator transform_iterator, const Polygon_2 &polygon)
    : polygon(polygon),
        transform_iterator(transform_iterator)
    {}

    Polygon_2_TransformsIterator(const Polygon_2_TransformsIterator &source)
    : polygon(source.polygon),
        transform_iterator(source.transform_iterator)
    {}

    inline Polygon_2_TransformsIterator &operator++()
    {
        transform_iterator++;

        return *this;
    }

    inline Polygon_2_TransformsIterator &operator--()
    {
        transform_iterator--;

        return *this;
    }

    inline Polygon_2_TransformsIterator &operator++(int)
    {
        Polygon_2_TransformsIterator res(*this);

        transform_iterator++;

        return res;
    }

    inline bool operator==(const Polygon_2_TransformsIterator &rhs) const
    {
        return transform_iterator == rhs.transform_iterator;
    }

    inline bool operator!=(const Polygon_2_TransformsIterator &rhs) const
    {
        return transform_iterator != rhs.transform_iterator;
    }

    inline const Polygon_2 operator*() const
    {
        return transform(*transform_iterator, polygon);
    }

    inline int steps_left() const
    {
        return transform_iterator.steps_left();
    }
};

}

#endif // _CPP_CAD_POLYGON_2_TRANSFORMS_ITERATOR_CPP
