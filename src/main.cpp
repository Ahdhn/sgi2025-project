#include <Eigen/Core>

#include "igl/read_triangle_mesh.h"
#include "polyscope/surface_mesh.h"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << R"(No input specified: "./LocRemesh mesh.ext" )";
        return 0;
    }

    polyscope::init();

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    std::string input = argv[1];

    igl::read_triangle_mesh(input, V, F);

    polyscope::registerSurfaceMesh(input, V, F);

    polyscope::show();
}
