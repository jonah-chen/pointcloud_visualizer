
#pragma once
#include <glm/glm.hpp>
#include <functional>
namespace cmap {
using fn = std::function<glm::vec3(float)>;
inline glm::vec3 red(float i) { return glm::vec3(1.0f, 0.0f, 0.0f); }
inline glm::vec3 jet(float i) 
{
    glm::vec3 c (1.0f, 1.0f, 1.0f);
    if (i < 0.25f) {
        c.r = 0.0f;
        c.g = 4.0f * i;
    } else if (i < 0.5f) {
        c.r = 0.0f;
        c.b = 1.0f - 4.0f * i;
    } else if (i < 0.75f) {
        c.r = 4.0f * i - 0.5f;
        c.b = 0.0f;
    } else {
        c.g = 1.0f - 4.0f * i;
        c.b = 0.0f;
    }
    return c;
}

} // namespace cmap
