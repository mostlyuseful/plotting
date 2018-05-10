#ifndef SLOPE_HPP
#define SLOPE_HPP

#include <boost/variant.hpp>

struct HorizontalSlope {};

struct FiniteSlope {
    double value;
};

using Slope = boost::variant<FiniteSlope, HorizontalSlope>;

class holds_finite_slope_visitor
    : public boost::static_visitor<bool>
{
public:

    bool operator()(HorizontalSlope const& s) const
    {
        return false;
    }

    bool operator()(FiniteSlope const& s) const
    {
        return true;
    }

};

#endif