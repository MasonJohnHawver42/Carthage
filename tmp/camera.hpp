// #pragma once

// #include <glm/glm.hpp>
// #include <glm/gtx/matrix_interpolation.hpp>

// namespace game 
// {

//     struct Camera 
//     {
//         glm::vec3 m_pos, m_dir, m_up;
//         float m_vfov, m_near, m_far, m_aspect;

//         glm::mat4 m_view, m_perspective, m_vp;
//         bool m_dirty_v, m_dirty_p;

//         Camera(glm::vec3 pos, glm::vec3 view, glm::vec3 up, float vfov, float near, float far, float aspect)
//             : m_pos(pos), m_dir(view), m_up(up), m_vfov(vfov), m_near(near), m_far(far), m_aspect(aspect), m_dirty_v(true), m_dirty_p(true) {}
    
//         void update_mats();
//     };

//     struct DirectionalLight 
//     {
//         glm::vec3 m_dir;
//         float m_intensity;
//     };

//     void Camera::update_mats()
//     {
//         bool updated = false;
//         if (m_dirty_v) 
//         {
//             glm::vec3 fwd = glm::normalize(-1.0f * m_dir);
//             glm::vec3 right = glm::normalize(glm::cross(m_dir, m_up));
//             glm::vec3 up = glm::normalize(glm::cross(right, m_dir));

//             m_view[0][0] = right[0]; m_view[1][0] = right[1]; m_view[2][0] = right[2]; m_view[3][0] = -1.0f * glm::dot(m_pos, right);
//             m_view[0][1] = up[0];    m_view[1][1] = up[1];    m_view[2][1] = up[2];    m_view[3][1] = -1.0f * glm::dot(m_pos, up);
//             m_view[0][2] = fwd[0];   m_view[1][2] = fwd[1];   m_view[2][2] = fwd[2];   m_view[3][2] = -1.0f * glm::dot(m_pos, fwd);
//             m_view[0][3] = 0.0f;     m_view[1][3] = 0.0f;     m_view[2][3] = 0.0f;     m_view[3][3] = 1.0f;

//             m_dirty_v = false;
//             updated = true;
//         }

//         if(m_dirty_p) 
//         {
//             float a = 1.0f / glm::tan(glm::radians(m_vfov) / 2.0f);
//             float b = -1.0f / (m_far - m_near);

//             m_perspective[0][0] = a / m_aspect;     m_perspective[0][1] = 0.0f; m_perspective[0][2] = 0.0f;                      m_perspective[0][3] = 0.0f;
//             m_perspective[1][0] = 0.0f;             m_perspective[1][1] = a;    m_perspective[1][2] = 0.0f;                      m_perspective[1][3] = 0.0f;
//             m_perspective[2][0] = 0.0f;             m_perspective[2][1] = 0.0f; m_perspective[2][2] = (m_far + m_near) * b;      m_perspective[2][3] = -1.0f;
//             m_perspective[3][0] = 0.0f;             m_perspective[3][1] = 0.0f; m_perspective[3][2] = 2.0f * m_far * m_near * b; m_perspective[3][3] = 0.0f;
            
//             m_dirty_p = false;
//             updated = true;
//         }

//         if (updated) 
//         {
//             m_vp = m_perspective * m_view;
//         }
//     }

//     struct CameraOperator 
//     {
//         int mode;
//         double xpos0, ypos0;

//         float speed, angular_speed; 

//         CameraOperator(float s, float as) : mode(0), xpos0(0), ypos0(0), speed(s), angular_speed(as) {}

//         void update_cam(float dt, double xpos1, double ypos1, bool click, bool forward, bool backward, 
//                         bool rightward, bool leftward, 
//                         bool upward, bool downward, bool input_captured, int width, int height, bool* setpos, int* nx, int* ny, bool* show, bool* hide, Camera& cam);
//     };

//     void operate_camera(float dt, float speed, float angular_speed, 
//                         float dx, float dy, 
//                         bool forward, bool backward, 
//                         bool rightward, bool leftward, 
//                         bool upward, bool downward, Camera& cam) 
//     {
//         glm::vec3 right = glm::normalize(glm::cross(cam.m_dir, cam.m_up));
//         glm::vec3 normal = glm::cross(cam.m_up, right);

//         float before = glm::dot(normal, cam.m_dir);

//         if (forward)   {cam.m_pos += 1.0f * dt * speed * cam.m_dir; }
//         if (backward)  {cam.m_pos += -1.0f * dt * speed * cam.m_dir; }
//         if (rightward) {cam.m_pos += 1.0f * dt * speed * right; }
//         if (leftward)  {cam.m_pos += -1.0f * dt * speed * right; }
//         if (upward)    {cam.m_pos += 1.0f * dt * speed * cam.m_up; }
//         if (downward)  {cam.m_pos += -1.0f * dt * speed * cam.m_up; }

//         cam.m_dir = glm::axisAngleMatrix(right, -1.0f * dy * angular_speed * dt) * glm::axisAngleMatrix(cam.m_up, -1.0f * dx * angular_speed * dt) * glm::vec4(cam.m_dir, 1.0f);

//         float after = glm::dot(normal, cam.m_dir);

//         if (before * after < 0.0f) 
//         {
//             cam.m_up = cam.m_up * -1.0f;
//         }

//         cam.m_dirty_v = true;
//     }

//     void CameraOperator::update_cam(float dt, double xpos1, double ypos1, bool click, bool forward, bool backward, 
//                         bool rightward, bool leftward, 
//                         bool upward, bool downward, bool input_captured, int width, int height, bool* setpos, int* nx, int* ny, bool* show, bool* hide, Camera& cam) 
//     {
//         *setpos = 0;
//         *hide = 0;
//         *show = 0;
//         if (mode == 2) 
//         {
//             // double xpos1, ypos1;
//             // glfwGetCursorPos(window, &xpos1, &ypos1);

//             operate_camera(dt, speed, angular_speed, xpos1 - xpos0, ypos1 - ypos0,
//                 forward, backward, rightward, leftward, upward, downward, cam);
            
//             *setpos = 1;
//             *nx = width >> 1; *ny = height >> 1;
//             xpos0 = width >> 1; ypos0 = height >> 1;
//         }

//         if (mode == 0 && click && !input_captured) 
//         {
//             mode = 1;
//             *hide = 1;
//         }

//         if (mode == 1 && !click) 
//         {
//             mode = 2;
//             xpos0 = xpos1;
//             ypos0 = ypos1;
//         }

//         if (mode == 2 && click) 
//         {
//             mode = 3;
//             *show = 1;
//         }

//         if (mode == 3 && !click) 
//         {
//             mode = 0;
//         }
//     }
