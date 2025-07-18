#include "botschRemesher.h"
#include "indicatorFunctions.h"
#include "mesh.h"
#include "utils.h"

namespace locremesh {

/**
 * Sets the feature vector for the remesh_botsch function.
 * It is essentially the list of indices of the vertices we don't want to be
 * altered during the remeshing.
 */
void BotschRemesher::setFeature()
{
    auto targetMesh = m_vertexSelector.getTargetMesh();

    std::set<int> verticesToIgnore;
    auto          selectedVerticesBitMask =
        m_vertexSelector.getSelectedVerticesBitMask();
    auto boundaryBitMask = targetMesh.getBoundaryBitMask();
    for (int i = 0; i < selectedVerticesBitMask.size(); i++) {
        if (!selectedVerticesBitMask[i] || boundaryBitMask[i]) {
            verticesToIgnore.insert(i);
        }
    }

    m_feature.resize(verticesToIgnore.size());
    for (int i = 0; auto idx : verticesToIgnore) {
        m_feature[i++] = idx;
    }
}

void BotschRemesher::remesh(std::string resultingMeshPolyscopeID)
{
    setFeature();

    auto targetMesh = m_vertexSelector.getTargetMesh();

    if (m_feature.size() == targetMesh.getVertexCount()) {
        throw std::runtime_error("No vertices were selected for remeshing.");
    }

    Eigen::VectorXd targetEdgeLengthsVector = Eigen::VectorXd::Constant(
        targetMesh.getVertexCount(), m_targetEdgeLength);

    // Create a copy of the mesh
    if (m_keepOriginalMesh) {
        m_resultingMesh = Mesh(targetMesh);
    }

    m_resultingMesh.setPolyscopeID(resultingMeshPolyscopeID);
    remesh_botsch(m_resultingMesh.getVertices(),
                  m_resultingMesh.getFaces(),
                  targetEdgeLengthsVector,
                  m_iterations,
                  m_feature,
                  m_shouldProject);
    m_resultingMesh.calculateMeshQuality();
    m_resultingMesh.calculateUVParametrization();
}

void BotschRemesher::polyscopeUISection()
{
    auto targetMesh = m_vertexSelector.getTargetMesh();

    ImGui::Text("Remeshing");
    ImGui::SliderFloat("Target Edge Length", &m_targetEdgeLength, 0.01f, 1.f);
    ImGui::SliderInt("Iterations", &m_iterations, 1, 100);
    ImGui::Checkbox("Project resulting mesh onto the original",
                    &m_shouldProject);
    ImGui::Checkbox("Keep original mesh", &m_keepOriginalMesh);

    if (ImGui::Button("Remesh")) {
        try {
            std::cout << "Running remesh_botsch..." << std::endl;
            remesh();
            std::cout << "Finished remesh_botsch" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error during remeshing: " << e.what() << std::endl;
            return;
        }
        // Update polyscope mesh
        auto sm = m_resultingMesh.polyscopeRegisterSurfaceMesh();
        if (m_keepOriginalMesh) {
            sm->setPosition({0, 0, 1});
        }
    }

    if (ImGui::Button("Reset")) {
        if (polyscope::hasSurfaceMesh(m_resultingMesh.getPolyscopeID())) {
            polyscope::removeSurfaceMesh(m_resultingMesh.getPolyscopeID());
        }
    }
}
}  // namespace locremesh