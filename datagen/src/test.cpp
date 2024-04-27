#include "core/containers.hpp"
#include <iostream>

#include "game/drone.hpp"

int main() {

    game::Quadrotor quadrotor;
    game::Controller controller;    

    quadrotor.reset(1, 0, 0, 0, 0, 0);
    controller.reset();

    for (int i = 0; i < 13; i++) { printf("%f ", quadrotor.state[i]); }
    printf("\n");

    double F, dt;
    Eigen::Vector3d M, dp, dv, da, dj;
    double dy, dyd;

    dp << 0, 0, 0;
    dv << 0, 0, 0;
    da << 0, 0, 0;
    dj << 0, 0, 0;
    dy = 10;
    dyd = 0;
    dt = .01;

    for (int k = 0; k < 5; k++) 
    {
        printf("ITER %d\n", k);
        controller.run(quadrotor, dp, dv, da, dj, dy, dyd, dt, F, M);

        std::cout << "f " << F << std::endl;
        std::cout << "M " << M << std::endl;

        if (std::isnan(F)) { exit(1); }

        quadrotor.update(dt, F, M);

        for (int i = 0; i < 13; i++) { printf("%f ", quadrotor.state[i]); }
        printf("\n");
    }

    return 0;
}