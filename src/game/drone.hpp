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
                        const std::function<bool(int*, float)>& solid, float* ds, 
                        const std::function<void(float*, unsigned int*)>& wtv, 
                        const std::function<void(unsigned int*, float*)>& vtw,
                        unsigned int wypt_steps, unsigned int time_steps, float lr); 

    void draw_route(const Eigen::MatrixXd &wayPs, game::DebugRenderer* dr, double r1, double r2, float* color);
    void draw_traj(min_snap::Trajectory& traj, unsigned int N, double r2, float* color, double start, double end, game::DebugRenderer* dr);

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
        
        void run(const Quadrotor& quad_state,
                 Eigen::Vector3d& desired_pos, 
                 Eigen::Vector3d& desired_vel,
                 Eigen::Vector3d& desired_acc,
                 Eigen::Vector3d& desired_jerk,
                 double desired_yaw, double desired_yawdot,
                 double dt, double& U, Eigen::Vector3d& M);
    };

    //python code:

    //state from quadrotor in python
    // def get_state(self):
    //     return State(self.state[0:3],
    //                  self.state[3:6],
    //                  RotToRPY(self.Rotation()),
    //                  self.state[10:13])


//    def run(state, des_state):
//     yaw_des = des_state.yaw
//     dyaw_des = des_state.yawdot

//     #current Rotation from body frame to world
//     bRw = RPYToRot(*state.rot).T
//     #normal vector to body frame
//     ZB = bRw[:,2]

//     #position and velocity errors
//     ep = des_state.pos - state.pos
//     ev = des_state.vel - state.vel

//     #command acceleration
//     commd_acc = kp*ep + kv*ev + des_state.acc
//     # Desired Thrust vector
//     F_des = m*commd_acc + m*g*np.array([0,0,1])
//     # Desired Thrust in ZB direction
//     U = F_des@ZB

//     #current acceleration
//     curr_acc = U*ZB/m - g*np.array([0,0,1])
//     #acceleration error
//     ea = curr_acc - des_state.acc

//     #command jerk
//     commd_jerk = kp*ev + kv*ea + des_state.jerk
//     # derivative of desired Thrust vector
//     dF_des = m*commd_jerk
//     # derivative of desired Thrust vector in ZB direction
//     dU = dF_des@ZB

//     #desired direction of normal vector
//     ZB_des = F_des/np.linalg.norm(F_des)

//     Xc = np.array([cos(yaw_des),sin(yaw_des),0])
//     ZB_Xc = np.cross(ZB_des,Xc)

//     YB_des = ZB_Xc/np.linalg.norm(ZB_Xc)
//     XB_des = np.cross(YB_des,ZB_des)

//     #desired Rotation matrix
//     R_des = np.c_[XB_des.T,YB_des.T,ZB_des.T]

//     #Rotation error
//     eR = 0.5*vee(bRw.T@R_des - R_des.T@bRw)

//     #projection of angular velocity on xB − yB plane
//     hw = (m*commd_jerk - dU*ZB_des)/U
//     #desired angular velocity
//     omega_des = np.array([-hw@YB_des,
//                            hw@XB_des,
//                            dyaw_des*ZB_des[2]])

//     #angular velocity error
//     eW = omega_des - state.omega

//     #moment
//     M = np.array([kR*eR + kW*eW]).T

//     return U, M




}