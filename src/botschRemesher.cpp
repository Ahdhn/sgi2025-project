#include "botschRemesher.h"
#include "indicatorFunctions.h"
#include "mesh.h"
#include "utils.h"

namespace locremesh {

/**
 * Sets the feature vector for the remesh_botsch function.
 * It is essentially the list of indices of the vertices we don't want to be
 * altered during the remeshing.
 *
 * @param targetMesh The
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

void BotschRemesher::remesh()
{
    setFeature();

    auto targetMesh = m_vertexSelector.getTargetMesh();

    if (m_feature.size() == targetMesh.getVertices().rows()) {
        throw std::runtime_error("No vertices were selected for remeshing.");
    }

    Eigen::VectorXd targetEdgeLengthsVector = Eigen::VectorXd::Constant(
        targetMesh.getVertices().rows(), m_targetEdgeLength);

    m_resultingMesh = targetMesh;
    remesh_botsch(m_resultingMesh.getVertices(),
                  m_resultingMesh.getFaces(),
                  targetEdgeLengthsVector,
                  m_iterations,
                  m_feature,
                  m_shouldProject);
}

void BotschRemesher::polyscopeUISection()
{
    auto targetMesh = m_vertexSelector.getTargetMesh();

    ImGui::Text("Remeshing");
    ImGui::SliderFloat("Target Edge Length", &m_targetEdgeLength, 0.01f, 1.f);
    ImGui::SliderInt("Iterations", &m_iterations, 1, 100);
    ImGui::Checkbox("Project resulting mesh onto the original",
                    &m_shouldProject);

    if (ImGui::Button("Remesh")) {
        try {
            std::cout << "Running remesh_botsch..." << std::endl;
            remesh();
            std::cout << "Finished remesh_botsch" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Error during remeshing: " << e.what() << std::endl;
            return;
        }

        // Use polyscope to render the resulting mesh

        if (polyscope::hasSurfaceMesh(m_polyscopeID)) {
            polyscope::removeSurfaceMesh(m_polyscopeID);
        }

        auto sm = polyscope::registerSurfaceMesh(m_polyscopeID,
                                                 m_resultingMesh.getVertices(),
                                                 m_resultingMesh.getFaces());
        sm->setEdgeWidth(1.0);
        sm->setPosition(glm::vec3(1.5f, 0.0f, 0.0f));

        auto fs = sm->addFaceScalarQuantity(
            "Quality",
            indFuncTriangleQuality(m_resultingMesh.getVertices(),
                                   m_resultingMesh.getFaces()));
        fs->setMapRange(std::make_pair<int, int>(0, 1));
        fs->setEnabled(true);
    }

    if (ImGui::Button("Reset")) {
        if (polyscope::hasSurfaceMesh(m_polyscopeID)) {
            polyscope::removeSurfaceMesh(m_polyscopeID);
        }
    }
}
}  // namespace locremesh