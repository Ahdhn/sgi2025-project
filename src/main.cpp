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
    locremesh::Mesh inputMesh(inputMeshFilename, "inputMesh");

    // The VertexSelector takes the Mesh and then handles the selection of
    // vertices that must be included in the remeshing stage.
    // It handles both the UI selection and the automated vertex selection.
    locremesh::VertexSelector vertexSelector(inputMesh);

    // The BotschRemeser takes the VertexSelector and uses it to drive the
    // remeshing procedures.
    locremesh::BotschRemesher botschRemesher(vertexSelector, 0.05, 10, true);

    // Polyscope Callback //////////////////////////////////////////////////////
    // This polyscope callback is where all UI related things are dealt with.
    polyscope::state::userCallback = [&]() {
        vertexSelector.handleManualVertexSelection(ImGui::GetIO());

        vertexSelector.polyscopeUISection();
        botschRemesher.polyscopeUISection();

        if (vertexSelector.wasSelectionModified())
            vertexSelector.updateSelectedVertices();
    };

    // Polyscope loop //////////////////////////////////////////////////////////
    polyscope::init();

    // Register the initial mesh in polyscope
    inputMesh.polyscopeRegisterSurfaceMesh();
    // equivalent to polyscope::show()
    while (!polyscope::windowRequestsClose()) {
        // Here is where the simulation will take place.

        // run mass-spring simulation on the mesh...
        // runMassSpringTimestep(inputMesh);

        // update mesh...
        // inputMesh.calculateMeshQuality();
        // inputMesh.calculateUVParametrization();

        // run the remeshing based on quality...
        // vertexSelector.selectVerticesBasedOnQuality();
        // botschRemesher.remesh();
        // inputMesh.polyscopeRegisterSurfaceMesh(); // Temporary

        // Update polyscope
        polyscope::frameTick();
    }

    return 0;
}
