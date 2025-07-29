#include <Eigen/Core>
#include <chrono>
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
    if (argc == 1) {
        std::cout << R"(No input specified: "./LocRemesh mesh.ext" )";
        return 0;
    }

    // Configurations //////////////////////////////////////////////////////////
    std::string inputMeshFilename = argv[1];
    std::string inputTextureFilename;
    if (argc > 2) {
        inputTextureFilename = argv[2];
    }

    float defaultQualityThreshold = 0.4;
    float defaultTargetEdgeLength = 0.06;
    int   defaultNumIterations    = 10;

    // The Mesh holds the input mesh data
    locremesh::Mesh inputMesh(
        inputMeshFilename, inputTextureFilename, "inputMesh");

    // The VertexSelector takes the Mesh and then handles the selection of
    // vertices that must be included in the remeshing stage.
    // It handles both the UI selection and the automated vertex selection.
    locremesh::VertexSelector vertexSelector(inputMesh,
                                             defaultQualityThreshold);

    // The BotschRemeser takes the VertexSelector and uses it to drive the
    // remeshing procedures.
    locremesh::BotschRemesher botschRemesher(
        vertexSelector, defaultTargetEdgeLength, defaultNumIterations, true);

    // Polyscope Callback //////////////////////////////////////////////////////
    // Physics simulation variables
    float  spatialFrequency             = 15.f;
    float  updatesPerSecond             = 50.f;
    float  amplitude                    = 0.1f;
    int    numMaxParamIterations        = 15;
    int    numOneRingDilationIterations = 5;
    bool   autoRemeshing                = false;
    bool   autoParametrization          = false;
    bool   runDeformation               = false;
    double accumulator                  = 0.0;
    double simulatedTime                = 0.0;

    polyscope::state::userCallback = [&]() {
        double dt = 1.f / updatesPerSecond;

        // Fixed timestep physics update.
        ImGui::Text("Stats");
        ImGui::Text("Simulated Time: %.2f", simulatedTime);
        ImGui::Separator();

        ImGui::Text("Deformation");
        ImGui::Checkbox("Run sinewave deformation", &runDeformation);
        ImGui::SliderFloat("Updates per second", &updatesPerSecond, 1.f, 100.f);
        ImGui::SliderFloat("Spatial Frequency", &spatialFrequency, 0.f, 100.f);
        ImGui::SliderFloat("Amplitude", &amplitude, 0.f, 1.f);

        ImGui::Checkbox("Auto Remesh", &autoRemeshing);
        ImGui::Checkbox("Auto Parametrization", &autoParametrization);
        if (ImGui::SliderInt(
                "Max Param Iterations", &numMaxParamIterations, 1, 15)) {
            inputMesh.setParamatrizationIterations(numMaxParamIterations);
        }
        ImGui::SliderInt(
            "One-ring Dilation degree", &numOneRingDilationIterations, 1, 10);

        // The accumulator is used to ensure that the physics simulation is
        // updated at a fixed rate, regardless of the frame rate.
        accumulator += ImGui::GetIO().DeltaTime;
        while (accumulator >= dt && runDeformation) {
            simulatedTime += dt;
            // Here is where the simulation will take place.
            std::cout << "Update timestep: " << accumulator << std::endl;

            if (autoRemeshing) {
                // Run remeshing for selected vertices in the previous update.
                botschRemesher.remesh();
                inputMesh.calculateUVParametrization(false);
            }

            // While the mass-spring simulation isn't ready, we move the mesh
            // by going through the vertices and updating their z position
            // according to a sine function
            Eigen::MatrixXd newVertices = inputMesh.getVertices();
            for (int i = 0; i < newVertices.rows(); ++i) {
                newVertices(i, 2) =
                    amplitude * std::sin(newVertices(i, 0) * spatialFrequency +
                                         simulatedTime / 1.5);
            }

            inputMesh.updateVertexPositions(newVertices);
            inputMesh.calculateMeshQuality();

            if (autoRemeshing) {
                vertexSelector.selectVerticesBasedOnQuality();
                for (int i = 0; i < numOneRingDilationIterations; ++i) {
                    vertexSelector.applyOneRingDilation();
                }
            }

            if (autoParametrization && !autoRemeshing) {
                inputMesh.calculateUVParametrization(true);
            }

            // Re-register the entire polyscope surface. (Not ideal but will do
            // for now)
            inputMesh.polyscopeRegisterSurfaceMesh();

            // run the remeshing based on quality..
            // botschRemesher.remesh();

            accumulator = 0;

            std::cout << std::endl;
        }
        ImGui::Separator();

        // vertexSelector.handleManualVertexSelection(ImGui::GetIO());

        vertexSelector.polyscopeUpdatePointCloud();
        vertexSelector.polyscopeUISection();
        botschRemesher.polyscopeUISection();

        // if (vertexSelector.getWasSelectionModified())
        //     vertexSelector.polyscopeUpdatePointCloud();
        polyscope::options::automaticallyComputeSceneExtents = false;
    };

    // Polyscope loop //////////////////////////////////////////////////////////
    polyscope::init();


    inputMesh.polyscopeRegisterSurfaceMesh();

    polyscope::show();

    return 0;
}
