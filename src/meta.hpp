#pragma once

#include <string>

inline const std::string getSrcDir()
{
    return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
};
