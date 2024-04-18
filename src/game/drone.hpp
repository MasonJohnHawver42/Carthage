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

        Eigen::Vector3d pos();
        Eigen::Matrix3d rot();

        void grad_state(double *dstate, double F, const Eigen::Vector3d& M);
    };

    // python code:

    // def __init__(self, pos, attitude):
    //     """ pos = [x,y,z] attitude = [rool,pitch,yaw]
    //         """
    //     self.state = np.zeros(13)
    //     roll, pitch, yaw = attitude
    //     rot    = RPYToRot(roll, pitch, yaw)
    //     quat   = RotToQuat(rot)
    //     self.state[0] = pos[0]
    //     self.state[1] = pos[1]
    //     self.state[2] = pos[2]
    //     self.state[6] = quat[0]
    //     self.state[7] = quat[1]
    //     self.state[8] = quat[2]
    //     self.state[9] = quat[3]
    //     self.ode = integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
    //     self.n_steps = 10
    //     self.t = 0


    struct Controller 
    {

        Eigen::Vector3d kv, kp, kR, kW, eI;

    };




}