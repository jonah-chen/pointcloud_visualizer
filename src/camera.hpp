
#pragma once

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera
{
public:
    Camera(const glm::vec3 &pos,
           const glm::vec3 &fwd,
           const glm::vec3 &up,
           float fov,
           float aspect,
           float zNear,
           float zFar,
           float ground_level);

    /**
     * Rotates the camera.
     * 
     * @param pitch float of pitch angle in radians
     * @param yaw float of yaw angle in radians
     */
    void rotate(float pitch, float yaw);

    /**
     * Translate the camera. If we attempt to translate the camera below the 
     * ground level, the downward component of the translation is clamped to 
     * the ground level.
     * 
     * @param fwd float of distance in meters to move forward   
     * @param right float of distance in meters to move right
     * @param up float of distance in meters to move up
     */
    void translate(float fwd, float right, float up);

    
private:
    glm::mat4 proj_;            // projection matrix
    glm::vec3 pos_;             // position of camera
    glm::vec3 fwd_, up_, right_;// forward, up, right vectors

public:
    float ground_level;        // ground level

    constexpr static float EYE_HEIGHT = 1.4f;

public:
    constexpr glm::vec3 pos() const { return pos_; }
    constexpr glm::vec3 fwd() const { return fwd_; }
    constexpr glm::vec3 up() const { return up_; }
    constexpr glm::vec3 right() const { return right_; }
    
    inline glm::mat4 view_proj() const 
    { return proj_ * glm::lookAt(pos_, pos_ + fwd_, up_); }
};
