#include "botschRemesher.h"
#include "indicatorFunctions.h"
#include "mesh.h"
#include "utils.h"

namespace locremesh {

void BotschRemesher::remesh(std::string resultingMeshPolyscopeID)
{
    auto feature    = m_vertexSelector.extractFeatureFromSelection();
    auto targetMesh = m_vertexSelector.getTargetMesh();

    if (feature.size() == targetMesh.getVertexCount()) {
        std::cout << "No vertices to remesh" << std::endl;
        return;
    }

    Eigen::VectorXd targetEdgeLengthsVector = Eigen::VectorXd::Constant(
        targetMesh.getVertexCount(), m_targetEdgeLength);

    if (m_keepOriginalMesh) {
        // Create a copy of the mesh
        m_resultingMesh = Mesh(targetMesh);
        m_resultingMesh.setPolyscopeID(resultingMeshPolyscopeID);
    } else {
        m_resultingMesh = targetMesh;
    }

    std::cout << "Running remesh_botsch..." << std::endl;
    remesh_botsch(m_resultingMesh.getVertices(),
                  m_resultingMesh.getFaces(),
                  targetEdgeLengthsVector,
                  m_iterations,
                  feature,
                  m_shouldProject);
    m_resultingMesh.identifyBoundaryVertices();
    m_resultingMesh.calculateMeshQuality();
    std::cout << "Finished remesh_botsch" << std::endl;
}

void BotschRemesher::polyscopeUISection()
{
    auto targetMesh = m_vertexSelector.getTargetMesh();

    ImGui::Text("Remeshing");
    ImGui::SliderFloat("Target Edge Length", &m_targetEdgeLength, 0.01f, 1.f);
    ImGui::SliderInt("Iterations", &m_iterations, 1, 100);
    ImGui::Checkbox("Project resulting mesh onto the original",
                    &m_shouldProject);
    // ImGui::Checkbox("Keep original mesh", &m_keepOriginalMesh);

    // if (ImGui::Button("Remesh")) {
    //     try {
    //         remesh();
    //     } catch (const std::exception& e) {
    //         std::cerr << "Error during remeshing: " << e.what() << std::endl;
    //         return;
    //     }

    //     // Update polyscope mesh
    //     auto sm = m_resultingMesh.polyscopeRegisterSurfaceMesh();

    //     if (m_keepOriginalMesh) {
    //         sm->setPosition({0, 0, 1});
    //     } else {
    //         m_vertexSelector.updateTargetMesh(m_resultingMesh);
    //     }
    // }

    // if (ImGui::Button("Reset")) {
    //     if (m_resultingMesh.getPolyscopeID() !=
    //         m_vertexSelector.getTargetMesh().getPolyscopeID()) {
    //         if (polyscope::hasSurfaceMesh(m_resultingMesh.getPolyscopeID())) {
    //             polyscope::removeSurfaceMesh(m_resultingMesh.getPolyscopeID());
    //         }
    //     }
    // }
}
}  // namespace locremesh