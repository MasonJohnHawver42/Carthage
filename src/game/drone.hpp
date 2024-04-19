#pragma once

#include "game/scene.hpp"
#include "game/octree.hpp"

#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include "traj_min_snap.hpp"

namespace game 
{


    void build_route(PlanningCache& plan_cache, unsigned int end_index, std::function<void(unsigned int*, float*)> convert, Eigen::MatrixXd& route);
    void allocate_time(const Eigen::MatrixXd &wayPs, double vel, double acc, Eigen::VectorXd& durations);

    bool rebuild_route(min_snap::Trajectory& traj, 
                        const Eigen::MatrixXd &route, const Eigen::VectorXd& ts, 
                        std::function<void(float*, unsigned int*)> vox, 
                        std::function<bool(int*, float)> solid, float d_w, 
                        Eigen::MatrixXd& next_route); 

    bool find_traj(game::PlanningCache& plan_cache, min_snap::SnapOpt& snapOpt, min_snap::Trajectory& minSnapTraj, Eigen::MatrixXd& route, Eigen::VectorXd& ts, 
                        float *start, float *end, 
                        const std::function<bool(int*, float)>& solid, 
                        const std::function<void(float*, unsigned int*)>& wtv, 
                        const std::function<void(unsigned int*, float*)>& vtw,
                        unsigned int wypt_steps, unsigned int time_steps, float lr); 

    void draw_route(const Eigen::MatrixXd &wayPs, game::DebugRenderer* dr);
    void draw_traj(min_snap::Trajectory& traj, const Eigen::VectorXd& ts, unsigned int N, game::DebugRenderer* dr);

    struct Quadrotor 
    {
        //params
        double mass, g, arm_length, height, minF, maxF, km, kf, r;

        Eigen::Matrix4d A, invA;
        Eigen::Matrix3d I, invI;

        //state: pos | vel | rot | omega 
        double state[13]; //x, y, z, dx, dy, dz, qw, qx, qy, qz, p, q, r 

        Quadrotor();

        void reset(double x, double y, double z, double roll, double pitch, double yaw);

        Eigen::Vector3d pos() const;
        Eigen::Vector3d vel() const;
        Eigen::Quaterniond quat() const;
        Eigen::Matrix3d rot() const;
        Eigen::Vector3d omega() const;

        void grad_state(double *dstate, double F, const Eigen::Vector3d& M);
        void update(double dt, double F, const Eigen::Vector3d& M);
    };

    struct Controller 
    {

        Eigen::Vector3d kv, kp, kR, kW, eI;

        Controller();

        void reset();

        void run_hover(const Quadrotor& quad_state, 
                       Eigen::Vector3d& desired_pos, 
                       Eigen::Vector3d& desired_vel,
                       Eigen::Vector3d& desired_acc,
                       double desired_yaw,
                       double dt, double& U, Eigen::Vector3d& M);
    };

    //python code:

    //state from quadrotor in python
    // def get_state(self):
    //     return State(self.state[0:3],
    //                  self.state[3:6],
    //                  RotToRPY(self.Rotation()),
    //                  self.state[10:13])


    // def run_hover(state, des_state, dt):
    // yaw_des = des_state.yaw
    // global eI

    // #position and velocity errors
    // ep = des_state.pos - state.pos
    // ev = des_state.vel - state.vel
    // eI += ep*dt

    // #command acceleration
    // commd_acc = kp*ep + kv*ev + kp*eI + des_state.acc

    // #Desired Thrust in ZB direction
    // U = m*commd_acc[2] + m*g

    // #Rotation error
    // phi_des = 1/g*(commd_acc[0]*sin(yaw_des) - commd_acc[1]*cos(yaw_des))
    // theta_des = 1/g*(commd_acc[0]*cos(yaw_des) + commd_acc[1]*sin(yaw_des))
    // eR = np.array([phi_des,theta_des,yaw_des]) - state.rot

    // #desired angular velocity
    // omega_des = np.array([0,0,0])
    // #angular velocity error
    // eW = omega_des - state.omega;

    // #moment
    // M = np.array([kR*eR + kW*eW]).T

    // return U, M




}