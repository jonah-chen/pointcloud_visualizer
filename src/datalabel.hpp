
#pragma once

#include <array>
struct Scannet
{
    const char *operator[](std::size_t i) const;
    std::size_t operator[](const char *name) const;

private:
    constexpr static std::array<const char *, 20> class_names = 
    {{}};
};