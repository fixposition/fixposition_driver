// Wrapper to suppress warnings from ROS headers
#ifndef __FIXPOSITION_DRIVER_ROS1_EXT_EIGEN_CONVERSIONS_HPP__
#define __FIXPOSITION_DRIVER_ROS1_EXT_EIGEN_CONVERSIONS_HPP__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#if defined(__GNUC__) && (__GNUC__ >= 9)
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#include <eigen_conversions/eigen_msg.h>
#pragma GCC diagnostic pop
// See commentes in fp_common/ext/eigen_core.hpp
#if !EIGEN_VERSION_AT_LEAST(3, 4, 0) && defined(__GNUC__) && (__GNUC__ >= 9)
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif
#endif  // __FIXPOSITION_DRIVER_ROS1_EXT_EIGEN_CONVERSIONS_HPP__
