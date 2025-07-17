#include <Eigen/Core>
#include <iostream>

#include "igl/boundary_loop.h"
#include "igl/igl_inline.h"
#include "igl/read_triangle_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "remesh/src/remesh_botsch.h"

#include "botschRemesher.h"
#include "indicatorFunctions.h"
#include "mesh.h"
#include "utils.h"
#include "vertexSelector.h"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << R"(No input specified: "./LocRemesh mesh.ext" )";
        return 0;
    }

    // Configurations //////////////////////////////////////////////////////////
    std::string inputMeshFilename = argv[1];

    // The Mesh holds the input mesh data
    locremesh::Mesh inputMesh(inputMeshFilename, "input mesh");

    // The VertexSelector takes the mesh and then handles the selection of
    // vertices that must be included in the remeshing stage.
    // It handles both the UI selection and the automated vertex selection
    locremesh::VertexSelector vertexSelector(inputMesh, "selected vertices");

    // The BotschRemeser takes the VertexSelector and uses it
    // to drive the remeshing procedures.
    locremesh::BotschRemesher botschRemesher(
        vertexSelector, 0.05, 10, true, "remeshed mesh");

    // Polyscope ///////////////////////////////////////////////////////////////
    polyscope::init();

    // Register the initial mesh in polyscope
    inputMesh.polyscopeRegister();

    // This polyscope callback is where all UI related things are dealt with.
    polyscope::state::userCallback = [&]() {
        vertexSelector.handleManualVertexSelection(ImGui::GetIO());
        vertexSelector.polyscopeUISection();

        botschRemesher.polyscopeUISection();

        if (vertexSelector.wasSelectionModified())
            vertexSelector.updateSelectedVertices();
    };

    // Polyscope loop //////////////////////////////////////////////////////////
    // equivalent to polyscope::show()
    while (!polyscope::windowRequestsClose()) {
        // Here is where the simulation will take place.

        // run mass-spring simulation on the mesh...
        // run the parametrization...
        // run the remeshing...

        // Update polyscope
        polyscope::frameTick();
    }

    return 0;
}
