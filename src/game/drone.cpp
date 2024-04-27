#include "game/drone.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <iostream>

game::Quadrotor::Quadrotor() 
{
    mass = 0.18;
    g = 9.8;
    arm_length = 0.2;
    height = 0.05;

    I << 0.00025, 0, 2.55e-6,
         0, 0.000232, 0,
         2.55e-6, 0, 0.0003738;
        
    invI = I.inverse();

    minF = 0.0;
    maxF = 5.0 * mass * g;

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
    for (int i = 0; i < 13; i++) { state[i] = 0; }

    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    
    state[0] = x;
    state[1] = y;
    state[2] = z;
    state[6] = q.w();
    state[7] = q.x();
    state[8] = q.y();
    state[9] = q.z();
}

Eigen::Vector3d game::Quadrotor::pos() const 
{
    return Eigen::Vector3d(state[0], state[1], state[2]);
}

Eigen::Vector3d game::Quadrotor::vel() const 
{
    return Eigen::Vector3d(state[3], state[4], state[5]);
}

Eigen::Quaterniond game::Quadrotor::quat() const 
{
    return Eigen::Quaterniond(state[6], state[7], state[8], state[9]);
}

Eigen::Matrix3d game::Quadrotor::rot() const
{
    double qw = state[6], qx = state[7], qy = state[8], qz = state[9];
    Eigen::Quaterniond quat(qw, qx, qy, qz);
    quat.normalize();
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();
    return rotation_matrix;
}

Eigen::Vector3d game::Quadrotor::omega() const 
{
    return Eigen::Vector3d(state[10], state[11], state[12]);
}

void game::Quadrotor::grad_state(double *dstate, double F, const Eigen::Vector3d& M) 
{
    double x = state[0], y = state[1], z = state[2];
    double xdot = state[3], ydot = state[4], zdot = state[5];
    double qw = state[6], qx = state[7], qy = state[8], qz = state[9];
    double p = state[10], q = state[11], r = state[12];

    Eigen::Quaterniond quat(qw, qx, qy, qz);
    Eigen::Vector4d quat_coeffs(qw, qx, qy, qz);

    quat.normalize();
    Eigen::Matrix3d bRw = quat.toRotationMatrix(); // world to body rotation matrix
    Eigen::Matrix3d wRb = bRw.transpose(); // orthogonal matrix inverse = transpose

    Eigen::Vector3d accel = (1.0 / mass) * ((wRb * Eigen::Vector3d(0, 0, F)) - Eigen::Vector3d(0, 0, mass * g));

    double K_quat = 2.0;
    double quaterror = 1.0 - (qw * qw + qx * qx + qy * qy + qz * qz);
    Eigen::Matrix<double, 4, 4> Q;
    Q << 0, -p, -q, -r,
        p,  0,  -r, q,
        q, r,  0,  -p,
        r, -q, p, 0;

    Eigen::Vector4d qdot_vec = ((-0.5) * Q * quat_coeffs) + (K_quat * quaterror * quat_coeffs);
    
    // std::cout << "FUCK " << qdot_vec << std::endl;

    Eigen::Vector3d omega(p, q, r);
    Eigen::Vector3d pqrdot = invI * (M - omega.cross(I * omega));

    // std::cout << "M2 " << M << std::endl;
    // std::cout << "T " <<  omega.cross(I * omega) << std::endl;

    dstate[0] = xdot;
    dstate[1] = ydot;
    dstate[2] = zdot;
    dstate[3] = accel[0];
    dstate[4] = accel[1];
    dstate[5] = accel[2];
    dstate[6] = qdot_vec[0];
    dstate[7] = qdot_vec[1];
    dstate[8] = qdot_vec[2];
    dstate[9] = qdot_vec[3];
    dstate[10] = pqrdot[0];
    dstate[11] = pqrdot[1];
    dstate[12] = pqrdot[2];
}

void game::Quadrotor::update(double dt, double F, const Eigen::Vector3d& M) 
{
    double Mt = M(2);

    // Compute individual propeller thrusts
    Eigen::Vector4d prop_thrusts = invA * Eigen::Vector4d(F, M(0), M(1), M(2));
    
    for (int i = 0; i < 4; ++i) {
        prop_thrusts(i) = std::max(std::min(prop_thrusts(i), maxF/4), minF/4);
    }

    F = prop_thrusts.sum();

    // Compute torques
    Eigen::Vector3d M_computed = A.block<3, 4>(1, 0) * prop_thrusts;
    M_computed(2) = Mt;

    double dstate[13];
    grad_state(dstate, F, M_computed);
    
    for (int j = 0; j < 13; ++j) { state[j] += dstate[j] * dt; }
    // printf("\n");
}

game::Controller::Controller() { reset(); }

void game::Controller::reset() 
{
    kv << 10, 10, 10;
    kp << 30, 30, 30;
    kR << 5, 5, 5;
    kW << 0.05, 0.05, 0.1;
    eI.setZero();
}

void game::Controller::run_hover(const Quadrotor& quad_state,
                                Eigen::Vector3d& desired_pos,
                                Eigen::Vector3d& desired_vel,
                                Eigen::Vector3d& desired_acc,
                                double desired_yaw,
                                double dt, double& U, Eigen::Vector3d& M) 
{
    double yaw_des = desired_yaw;

    Eigen::Vector3d ep = desired_pos - quad_state.pos();
    Eigen::Vector3d ev = desired_vel - quad_state.vel();
    eI += ep * dt;

    Eigen::Vector3d commd_acc = kp.cwiseProduct(ep) + kv.cwiseProduct(ev) + kp.cwiseProduct(eI) + desired_acc;

    U = quad_state.mass * commd_acc(2) + quad_state.mass * quad_state.g;

    // Rotation error
    double phi_des = (1.0 / quad_state.g) * (commd_acc(0) * sin(yaw_des) - commd_acc(1) * cos(yaw_des));
    double theta_des = (1.0 / quad_state.g) * (commd_acc(0) * cos(yaw_des) + commd_acc(1) * sin(yaw_des));
    Eigen::Vector3d eR(phi_des, theta_des, yaw_des);
    Eigen::Vector3d state_rot = quad_state.rot().eulerAngles(0, 1, 2);
    Eigen::Vector3d eR_error = eR - state_rot;
    
    // Angular velocity error
    Eigen::Vector3d omega_des(0, 0, 0);
    Eigen::Vector3d eW = omega_des - quad_state.omega();

    // Moment
    M = kR.cwiseProduct(eR_error) + kW.cwiseProduct(eW);
}

Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(-S(1, 2), S(0, 2), -S(0, 1));
}

void game::Controller::run(const game::Quadrotor& quad_state,
                 Eigen::Vector3d& desired_pos, 
                 Eigen::Vector3d& desired_vel,
                 Eigen::Vector3d& desired_acc,
                 Eigen::Vector3d& desired_jerk,
                 double desired_yaw, double desired_yawdot,
                 double dt, double& U, Eigen::Vector3d& M) 
{

    double m = quad_state.mass;
    double g = quad_state.g;

    Eigen::Matrix3d bRw = quad_state.rot().transpose(); // current Rotation from body frame to world
    Eigen::Vector3d ZB = bRw.col(2); // normal vector to body frame

    Eigen::Vector3d ep = desired_pos - quad_state.pos(); // position error
    Eigen::Vector3d ev = desired_vel - quad_state.vel(); // velocity error

    Eigen::Vector3d commd_acc = kp.cwiseProduct(ep) + kv.cwiseProduct(ev) + desired_acc; // command acceleration
    Eigen::Vector3d F_des = (m * commd_acc) + (m * g * Eigen::Vector3d(0, 0, 1)); // Desired Thrust vector
    U = F_des.dot(ZB); // Desired Thrust in ZB direction

    Eigen::Vector3d curr_acc = (U * ZB / m) - (g * Eigen::Vector3d(0, 0, 1)); // current acceleration
    Eigen::Vector3d ea = curr_acc - desired_acc; // acceleration error

    // std::cout << "ZB " << bRw << std::endl;
    // std::cout << "curr_acc " << curr_acc << std::endl;
    // std::cout << "ea " << ea << std::endl;

    Eigen::Vector3d commd_jerk = kp.cwiseProduct(ev) + kv.cwiseProduct(ea) + desired_jerk; // command jerk
    Eigen::Vector3d dF_des = m * commd_jerk; // derivative of desired Thrust vector
    double dU = dF_des.dot(ZB); // derivative of desired Thrust vector in ZB direction

    // printf("dU %f\n", dU);
    // std::cout << dF_des << std::endl;

    Eigen::Vector3d ZB_des = F_des.normalized(); // desired direction of normal vector

    Eigen::Vector3d Xc(cos(desired_yaw), sin(desired_yaw), 0);
    Eigen::Vector3d ZB_Xc = ZB_des.cross(Xc);

    // std::cout << "xc " << Xc << std::endl;
    // std::cout << "zbxc " << ZB_Xc << std::endl;
    // std::cout << "zb_des " << ZB_des << std::endl;

    Eigen::Vector3d YB_des = ZB_Xc.normalized();
    Eigen::Vector3d XB_des = YB_des.cross(ZB_des);

    Eigen::Matrix3d R_des;
    R_des << XB_des, YB_des, ZB_des; // desired Rotation matrix

    // std::cout << "R_des" << R_des << std::endl;

    Eigen::Vector3d eR = 0.5 * vee(bRw.transpose() * R_des - R_des.transpose() * bRw); // Rotation error

    Eigen::Vector3d hw = (m * commd_jerk - dU * ZB_des) / U; // projection of angular velocity on xB − yB plane
    Eigen::Vector3d omega_des(-1 * hw.dot(YB_des), hw.dot(XB_des), desired_yawdot * ZB_des.z()); // desired angular velocity
    // std::cout << "omega_des " << omega_des << std::endl;
    
    Eigen::Vector3d eW = omega_des - quad_state.omega(); // angular velocity error

    M = kR.cwiseProduct(eR) + kW.cwiseProduct(eW); // moment
}


bool game::find_traj(game::PlanningCache& plan_cache, min_snap::SnapOpt& snapOpt, min_snap::Trajectory& minSnapTraj, Eigen::MatrixXd& route, Eigen::VectorXd& ts,
                        float *start, float *end, 
                        const std::function<bool(int*, float)>& solid, float* ds,
                        const std::function<void(float*, unsigned int*)>& wtv, 
                        const std::function<void(unsigned int*, float*)>& vtw,
                        unsigned int wypt_steps, unsigned int time_steps, float lr)
{
    
    unsigned vox_start[3], vox_end[3];
    wtv(start, vox_start); wtv(end, vox_end);

    unsigned int end_index = game::theta_star(plan_cache, vox_start, vox_end, solid, ds[0], ds[1]);
    printf("end_index %d\n", end_index);
    if (end_index == -1) { return false; }

    Eigen::MatrixXd next_route;
    Eigen::VectorXd min_ts, dts;
    Eigen::Matrix<double, 3, 4> iSS, fSS;

    game::build_route(plan_cache, end_index, vtw, route);
    // std::cout << "route " << route << std::endl;
    game::allocate_time(route, 3, 3, ts);
    min_ts = ts;

    iSS.setZero();
    fSS.setZero();

    iSS.col(0) << route.leftCols<1>();
    fSS.col(0) << route.rightCols<1>();

    float p = 1.0f;
    float q = 2.0f;

    for (int i = 0; i < wypt_steps; i++) 
    {
        for (int i = 0; i < time_steps; i++) 
        {
            snapOpt.reset(iSS, fSS, route.cols() - 1);
            snapOpt.generate(route.block(0, 1, 3, route.cols() - 2), ts);
            snapOpt.getTraj(minSnapTraj);

            dts = snapOpt.getGradT();

            for (int j = 0; j < dts.size(); j++) 
            {
                if (ts(j) < min_ts(j)) { printf("WTF\n"); }
                dts(j) += ts(j) >= min_ts(j) ? 0.0 : p * q * pow(ts(j) - min_ts(j), q - 1.0f);
            }

            ts -= lr * dts;
        }

        bool res = game::rebuild_route(minSnapTraj, route, ts, wtv, solid, ds[2], next_route);

        if (!res) { return true; }
        if (i + 1 < wypt_steps) 
        {
            route = next_route;
            game::allocate_time(route, 5, 3, ts);
            min_ts = ts;
        }
    }

    return false;
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


void game::draw_route(const Eigen::MatrixXd &route, game::DebugRenderer* dr, double r1, double r2, float* color) 
{
    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    for (int i = 1; i < route.cols(); i++) 
    {
        float pos0[] = {(float)route(0, i - 1), (float)route(1, i - 1), (float)route(2, i - 1)};
        float pos1[] = {(float)route(0, i), (float)route(1, i), (float)route(2, i)};
        add_line(pos0, pos1, r1, color, dr);

        ent = dr->shape(gfx::Shape::CUBE);
        trans_tmp = {{pos0[0], pos0[1], pos0[2]}, {0, 0, 1}, 0, {r2, r2, r2}};
        trans_tmp.mat4(ent->mat);
        game::set_color(0.0, 0.0, 0.0, 1, ent->color);
    }
}

void game::draw_traj(min_snap::Trajectory& traj, unsigned int N, double r2, float* color, double start, double end, game::DebugRenderer* dr) 
{
    double t_step = (end - start) / N, t = start;
    Eigen::Vector3d p0, p1;

    gfx::ShapeEntry* ent;
    game::Transform trans_tmp;

    while (t < end) 
    {
        p0 = traj.getPos(t);
        p1 = traj.getPos(t + t_step);

        float pos0[] = {(float)p0(0), (float)p0(1), (float)p0(2)};
        float pos1[] = {(float)p1(0), (float)p1(1), (float)p1(2)};

        add_line(pos0, pos1, r2, color, dr);

        // ent = dr->shape(gfx::Shape::CUBE);
        // trans_tmp = {{pos0[0], pos0[1], pos0[2]}, {0, 0, 1}, 0, {.1, .1, .1}};
        // trans_tmp.mat4(ent->mat);
        // game::set_color(0.0, 1.0, 0.0, 1, ent->color);

        t += t_step;
    }
}
