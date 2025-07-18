#include "mass_spring.h"
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>

int main() {
    polyscope::init();

    MassSpringSystem system;
    system.initializeGrid(10, 10);

    auto* pc = polyscope::registerPointCloud("Mass Nodes", system.x);

    polyscope::state::userCallback = [&]() {
        system.stepImplicitEuler();
        pc->updatePointPositions(system.x);
    };

    polyscope::show();
    return 0;
}
