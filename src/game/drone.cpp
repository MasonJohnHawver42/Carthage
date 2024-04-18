#include "game/drone.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <iostream>

game::Quadrotor::Quadrotor() 
{
    mass = 0.18;
    g = 9.8;
    arm_length = 0.086;
    height = 0.05;

    I << 0.00025, 0, 2.55e-6,
         0, 0.000232, 0,
         2.55e-6, 0, 0.0003738;
        
    invI = I.inverse();

    minF = 0.0;
    maxF = 2.0 * mass * g;

    km = 1.5e-9;
    kf = 6.11e-8;
    r = km / kf;

    double L = arm_length;
    A << 1,  1,  1,  1,
         0,  L,  0, -L,
         -L,  0,  L,  0,
         r, -r,  r, -r;
    
    invA = A.inverse();


    for (int i = 0; i < 13; i++) { state[i] = 0; }
}

void game::Quadrotor::reset(double x, double y, double z, double roll, double pitch, double yaw) 
{
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;
}

void game::build_route(PlanningCache& plan_cache, unsigned int end_index, std::function<void(unsigned int*, float*)> convert, Eigen::MatrixXd& route) 
{
    unsigned int curr = end_index, next, index, N = 0;
    while (curr != -1)
    {
        index = plan_cache.index(curr);
        next = (plan_cache.m_parent[index] >> 2) - 1;
        N++;
        curr = next;
    }

    printf("%d \n", N);

    route = Eigen::MatrixXd(3, N);

    curr = end_index;

    Eigen::Array3d temp;
    unsigned int v[3];
    float pos[3];

    while (curr != -1)
    {
        N--;
        index = plan_cache.index(curr);
        next = (plan_cache.m_parent[index] >> 2) - 1;
        
        plan_cache.vox(curr, v);
        convert(v, pos);

        temp << pos[0], pos[1], pos[2];
        route.col(N) << temp;
        
        curr = next;
    }
}

void game::allocate_time(const Eigen::MatrixXd &wayPs, double vel, double acc, Eigen::VectorXd& durations)
{
    int N = (int)(wayPs.cols()) - 1;
    durations = Eigen::VectorXd(N);
    if (N > 0)
    {

        Eigen::Vector3d p0, p1;
        double dtxyz, D, acct, accd, dcct, dccd, t1, t2, t3;
        for (int k = 0; k < N; k++)
        {
            p0 = wayPs.col(k);
            p1 = wayPs.col(k + 1);
            D = (p1 - p0).norm();

            acct = vel / acc;
            accd = (acc * acct * acct / 2);
            dcct = vel / acc;
            dccd = acc * dcct * dcct / 2;

            if (D < accd + dccd)
            {
                t1 = sqrt(acc * D) / acc;
                t2 = (acc * t1) / acc;
                dtxyz = t1 + t2;
            }
            else
            {
                t1 = acct;
                t2 = (D - accd - dccd) / vel;
                t3 = dcct;
                dtxyz = t1 + t2 + t3;
            }

            durations(k) = dtxyz;
        }
    }
}

bool game::rebuild_route(min_snap::Trajectory& traj, const Eigen::MatrixXd &route, const Eigen::VectorXd& ts, std::function<void(float*, unsigned int*)> vox, std::function<bool(int*, float)> solid, float d_w, Eigen::MatrixXd& next_route) 
{
    double max_time = ts.sum();
    double t_step = .1f, t = 0.0f;
    Eigen::Vector3d p0, p1;

    std::set<int> col_pieces;

    while (t < max_time) 
    {
        p0 = traj.getPos(t);
        p1 = traj.getPos(t + t_step);

        float pos0[] = {(float)p0(0), (float)p0(1), (float)p0(2)};
        float pos1[] = {(float)p1(0), (float)p1(1), (float)p1(2)};

        unsigned vox_p0[3], vox_p1[3];
        vox(pos0, vox_p0);
        vox(pos1, vox_p1);

        bool result = game::raycast(vox_p0, vox_p1, solid, 2.0f);

        if (result) 
        {
            double t0 = t; int idx_0 = traj.locatePieceIdx(t0);
            double t1 = t + t_step; int idx_1 = traj.locatePieceIdx(t1);
            col_pieces.insert(idx_0);
            col_pieces.insert(idx_1);
        }

        // printf("#");

        // add_line(pos0, pos1, 0.05f, &debug_renderer);

        // ent = debug_renderer.shape(gfx::Shape::CUBE);
        // trans_tmp = {{pos0[0], pos0[1], pos0[2]}, {0, 0, 1}, 0, {.1, .1, .1}};
        // trans_tmp.mat4(ent->mat);
        // game::set_color(result ? 1.0 : 0.0, result ? 0.0 : 1.0, 0.0, 1, ent->color);

        t += t_step;
    }

    // printf("\n");

    if (col_pieces.size() == 0) { return false; }

    next_route = Eigen::MatrixXd(3, route.cols() + col_pieces.size());

    // printf("%d %d\n", route.cols() + col_pieces.size(), col_pieces.size());

    auto it = col_pieces.begin();
    int j = 0;
    for (int i = 0; i < route.cols(); i++) 
    {
        // printf("%d %d\n", *it, j);
        if (it == col_pieces.end() || i <= *it) { next_route.col(j) << route.col(i); j++; }
        if (i == *it && i + 1 < route.cols()) 
        { 
            next_route.col(j) << ((route.col(i) + route.col(i + 1)) / 2.0);
            j++;
            ++it;
        }
    }

    return true;
}


void game::draw_route(const Eigen::MatrixXd &route, game::DebugRenderer* dr) 
{
    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    for (int i = 1; i < route.cols(); i++) 
    {
        float pos0[] = {(float)route(0, i - 1), (float)route(1, i - 1), (float)route(2, i - 1)};
        float pos1[] = {(float)route(0, i), (float)route(1, i), (float)route(2, i)};
        add_line(pos0, pos1, 0.025f, dr);

        ent = dr->shape(gfx::Shape::CUBE);
        trans_tmp = {{pos0[0], pos0[1], pos0[2]}, {0, 0, 1}, 0, {.1, .1, .1}};
        trans_tmp.mat4(ent->mat);
        game::set_color(0.0, 0.0, 0.0, 1, ent->color);
    }
}

void game::draw_traj(min_snap::Trajectory& traj, const Eigen::VectorXd& ts, unsigned int N, game::DebugRenderer* dr) 
{
    double max_time = ts.sum();
    double t_step = max_time / N, t = 0.0f;
    Eigen::Vector3d p0, p1;

    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    while (t < max_time) 
    {
        p0 = traj.getPos(t);
        p1 = traj.getPos(t + t_step);

        float pos0[] = {(float)p0(0), (float)p0(1), (float)p0(2)};
        float pos1[] = {(float)p1(0), (float)p1(1), (float)p1(2)};

        add_line(pos0, pos1, 0.05f, dr);

        ent = dr->shape(gfx::Shape::CUBE);
        trans_tmp = {{pos0[0], pos0[1], pos0[2]}, {0, 0, 1}, 0, {.1, .1, .1}};
        trans_tmp.mat4(ent->mat);
        game::set_color(0.0, 1.0, 0.0, 1, ent->color);

        t += t_step;
    }
}
