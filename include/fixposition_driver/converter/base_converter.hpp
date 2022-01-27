/**
 *  ___    ___
 *  \  \  /  /
 *   \  \/  /
 *   /  /\  \
 *  /__/  \__\  Fixposition AG
 *
 * @file base_converter.hpp
 * @author Kailin Huang (kailin.huang@fixposition.com)
 * @brief
 * @date 2022-01-26
 *
 */

#ifndef __FIXPOSITION_DRIVER_CONVERTER_BASE_CONVERTER_HPP__
#define __FIXPOSITION_DRIVER_CONVERTER_BASE_CONVERTER_HPP__

/* SYSTEM / STL */

/* EXTERNAL */

/* ROS */
#include <ros/console.h>

/* PACKAGE */

namespace fixposition {
class BaseConverter {
   public:
    BaseConverter() = default;
    ~BaseConverter() = default;

    void ConvertStringAndPublish(const std::string& state);
    virtual void ConvertTokensAndPublish(const std::vector<std::string>& tokens) = 0;
    virtual bool CheckHeader(const std::string msg_header) = 0;
};
}  // namespace fixposition
#endif