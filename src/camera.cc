
#include "camera.hpp"
#include <stdexcept>

constexpr float QUARTERNION_ERROR_TOLERANCE = 1e-3f;

Camera::Camera(const glm::vec3 &pos,
               const glm::vec3 &fwd,
               const glm::vec3 &up,
               float fov,
               float aspect,
               float zNear,
               float zFar,
               float ground_level)
    : pos_(pos),
      proj_(glm::perspective(fov, aspect, zNear, zFar)),
      ground_level_(ground_level)
{
    fwd_ = glm::normalize(fwd);
    up_ = glm::normalize(up);
    right_ = glm::cross(fwd_, up_);

    if (glm::dot(fwd_, up_) == 0.0f)
        throw std::runtime_error("Camera::Camera: fwd and up are parallel");
}

void Camera::rotate(float pitch, float yaw)
{
    glm::mat4 rotation = glm::mat4(1.0f);

	rotation = glm::rotate(rotation, pitch,
						   glm::vec3(right_.x, 0.0f, right_.z));
	rotation = glm::rotate(rotation, yaw, glm::vec3(0.0f, 1.0f, 0.0f));

	// quaternion rotations can cause floating point errors.
	// correct using the small angle approximation.
	if ((right_.y > QUARTERNION_ERROR_TOLERANCE
		 or right_.y < -QUARTERNION_ERROR_TOLERANCE) and up_.y > 0.5f)
		rotation = glm::rotate(rotation, right_.y,
							   glm::vec3(fwd_.x, 0.0f, fwd_.z));

	fwd_ = glm::normalize(glm::vec3(rotation * glm::vec4(fwd_, 1.0f)));
	up_ = glm::normalize(glm::vec3(rotation * glm::vec4(up_, 1.0f)));
	right_ = glm::normalize(glm::cross(fwd_, up_));
}

void Camera::translate(float fwd, float right, float up)
{
    pos_.x += fwd * fwd_.x + right * right_.x;
    pos_.y += up;
    pos_.z += fwd * fwd_.z + right * right_.z;

    if (pos_.y < ground_level_)
        pos_.y = ground_level_;
}


