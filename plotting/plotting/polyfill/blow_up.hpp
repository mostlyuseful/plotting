#ifndef BLOW_UP_HPP
#define BLOW_UP_HPP

#include "dpath.hpp"

#include <clipper.hpp>

ClipperLib::Path blow_up(ClipperLib::DPath const &input, double const factor);
ClipperLib::Paths blow_up(ClipperLib::DPaths const &input, double const factor);

#endif /* BLOW_UP_HPP */
