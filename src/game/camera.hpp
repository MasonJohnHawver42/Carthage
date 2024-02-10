#pragma once

#include <glm/glm.hpp>
#include <glm/gtx/matrix_interpolation.hpp>

namespace game 
{

    struct Camera 
    {
        glm::vec3 m_pos, m_view, m_up;
        float m_vfov, m_near, m_far, m_aspect;
    };

    void create_camera(glm::vec3 pos, glm::vec3 view, glm::vec3 up, float vfov, float near, float far, float aspect, Camera* cam) 
    {
        cam->m_pos = pos;
        cam->m_view = view;
        cam->m_up = up;
        cam->m_vfov = vfov;
        cam->m_near = near;
        cam->m_far = far;
        cam->m_aspect = aspect;
    }

    void view_matrix(Camera& cam, glm::mat4& mat) 
    {
        glm::vec3 fwd = glm::normalize(-1.0f * cam.m_view);
        glm::vec3 right = glm::normalize(glm::cross(cam.m_view, cam.m_up));
        glm::vec3 up = glm::normalize(glm::cross(right, cam.m_view));

        mat[0][0] = right[0]; mat[1][0] = right[1]; mat[2][0] = right[2]; mat[3][0] = -1.0f * glm::dot(cam.m_pos, right);
        mat[0][1] = up[0];    mat[1][1] = up[1];    mat[2][1] = up[2];    mat[3][1] = -1.0f * glm::dot(cam.m_pos, up);
        mat[0][2] = fwd[0];   mat[1][2] = fwd[1];   mat[2][2] = fwd[2];   mat[3][2] = -1.0f * glm::dot(cam.m_pos, fwd);
        mat[0][3] = 0.0f;     mat[1][3] = 0.0f;     mat[2][3] = 0.0f;     mat[3][3] = 1.0f;
    
    }

    void proj_matrix(Camera& cam, glm::mat4& mat) 
    {
        float a = 1.0f / glm::tan(glm::radians(cam.m_vfov) / 2.0f);
        float b = -1.0f / (cam.m_far - cam.m_near);

        mat[0][0] = a / cam.m_aspect; mat[0][1] = 0.0f; mat[0][2] = 0.0f;                              mat[0][3] = 0.0f;
        mat[1][0] = 0.0f;             mat[1][1] = a;    mat[1][2] = 0.0f;                              mat[1][3] = 0.0f;
        mat[2][0] = 0.0f;             mat[2][1] = 0.0f; mat[2][2] = (cam.m_far + cam.m_near) * b;      mat[2][3] = -1.0f;
        mat[3][0] = 0.0f;             mat[3][1] = 0.0f; mat[3][2] = 2.0f * cam.m_far * cam.m_near * b; mat[3][3] = 0.0f;
    }

    void operate_camera(float dt, float speed, float angular_speed, 
                        float dx, float dy, 
                        bool forward, bool backward, 
                        bool rightward, bool leftward, 
                        bool upward, bool downward, Camera& cam) 
    {
        glm::vec3 right = glm::normalize(glm::cross(cam.m_view, cam.m_up));
        glm::vec3 normal = glm::cross(cam.m_up, right);

        float before = glm::dot(normal, cam.m_view);

        if (forward)   {cam.m_pos += 1.0f * dt * speed * cam.m_view; }
        if (backward)  {cam.m_pos += -1.0f * dt * speed * cam.m_view; }
        if (rightward) {cam.m_pos += 1.0f * dt * speed * right; }
        if (leftward)  {cam.m_pos += -1.0f * dt * speed * right; }
        if (upward)    {cam.m_pos += 1.0f * dt * speed * cam.m_up; }
        if (downward)  {cam.m_pos += -1.0f * dt * speed * cam.m_up; }

        cam.m_view = glm::axisAngleMatrix(right, -1.0f * dy * angular_speed * dt) * glm::axisAngleMatrix(cam.m_up, -1.0f * dx * angular_speed * dt) * glm::vec4(cam.m_view, 1.0f);

        float after = glm::dot(normal, cam.m_view);

        if (before * after < 0.0f) 
        {
            cam.m_up = cam.m_up * -1.0f;
        }
    }
}