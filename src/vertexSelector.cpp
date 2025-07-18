#include "vertexSelector.h"

namespace locremesh {

void VertexSelector::selectVerticesBasedOnQuality()
{
    m_selectionBitMask.assign(m_targetMesh.getVertexCount(), false);
    const Eigen::VectorXd& quality = m_targetMesh.getQuality();
    for (int i = 0; i < quality.size(); ++i) {
        if (quality[i] <= m_qualityThreshold) {
            auto face                   = m_targetMesh.getFaces().row(i);
            m_selectionBitMask[face[0]] = true;
            m_selectionBitMask[face[1]] = true;
            m_selectionBitMask[face[2]] = true;
        }
    }
    m_wasSelectionModified = true;
}

void VertexSelector::updateSelectedVertices()
{
    // Clear previous data
    std::set<int> selectedVertices;
    for (int i = 0; i < m_selectionBitMask.size(); i++) {
        if (m_selectionBitMask[i]) {
            selectedVertices.insert(i);
        }
    }

    if (selectedVertices.empty()) {
        m_selectedVerticesStr = "None";
    } else {
        std::stringstream ss;
        auto              it = selectedVertices.begin();
        ss << *it;
        for (++it; it != selectedVertices.end(); ++it) {
            ss << ", " << *it;
        }
        m_selectedVerticesStr = ss.str();
    }


    if (polyscope::hasPointCloud(m_selectedVerticesPointCloudPSID)) {
        polyscope::removePointCloud(m_selectedVerticesPointCloudPSID);
    }
    if (!selectedVertices.empty()) {
        auto pc = polyscope::registerPointCloud(
            m_selectedVerticesPointCloudPSID,
            getMeshVerticesCoords(m_targetMesh.getVertices(),
                                  selectedVertices));
        pc->setEnabled(true);
        pc->setPointColor({1.0, 0.0, 0.0});
    }

    m_wasSelectionModified = false;
}

void VertexSelector::handleManualVertexSelection(ImGuiIO& io)
{
    // ALT + Click to select a vertex
    if (io.KeyAlt && io.MouseClicked[0]) {
        glm::vec2             screenCoords{io.MousePos.x, io.MousePos.y};
        polyscope::PickResult pickResult =
            polyscope::pickAtScreenCoords(screenCoords);

        auto targetSurfaceMesh =
            polyscope::getSurfaceMesh(m_targetMesh.getPolyscopeID());

        if (pickResult.isHit && pickResult.structure == targetSurfaceMesh) {
            polyscope::SurfaceMeshPickResult meshPickResult =
                targetSurfaceMesh->interpretPickResult(pickResult);

            if (meshPickResult.elementType == polyscope::MeshElement::VERTEX) {
                std::cout << "clicked vertex " << meshPickResult.index
                          << std::endl;

                m_selectionBitMask[meshPickResult.index] = true;

                if (!m_wasSelectionModified)
                    m_wasSelectionModified = true;
            }
        }
    }

    // CTRL + Click to select all vertices of a face
    if (io.KeyCtrl && io.MouseClicked[0]) {
        glm::vec2             screenCoords{io.MousePos.x, io.MousePos.y};
        polyscope::PickResult pickResult =
            polyscope::pickAtScreenCoords(screenCoords);

        auto targetSurfaceMesh =
            polyscope::getSurfaceMesh(m_targetMesh.getPolyscopeID());

        if (pickResult.isHit && pickResult.structure == targetSurfaceMesh) {
            polyscope::SurfaceMeshPickResult meshPickResult =
                targetSurfaceMesh->interpretPickResult(pickResult);

            if (meshPickResult.elementType == polyscope::MeshElement::FACE) {
                std::cout << "clicked face " << meshPickResult.index
                          << std::endl;

                Eigen::MatrixXi faces = m_targetMesh.getFaces();
                m_selectionBitMask[faces(meshPickResult.index, 0)] = true;
                m_selectionBitMask[faces(meshPickResult.index, 1)] = true;
                m_selectionBitMask[faces(meshPickResult.index, 2)] = true;

                if (!m_wasSelectionModified)
                    m_wasSelectionModified = true;
            }
        }
    }
}

void VertexSelector::polyscopeUISection()
{
    ImGui::Text("Vertex Selection");
    ImGui::SameLine();
    ImGui::TextDisabled("(help)");
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted("ALT+Click on a vertex to select it.");
        ImGui::TextUnformatted(
            "CTRL+Click on a face to select all its vertices.");
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }

    if (ImGui::CollapsingHeader("Selected Vertices")) {
        ImGui::TextWrapped("%s", m_selectedVerticesStr.c_str());
    }

    if (ImGui::Button("Select All")) {
        m_selectionBitMask.assign(m_targetMesh.getVertexCount(), true);
        m_wasSelectionModified = true;
    }

    if (ImGui::Button("Apply One-Ring Dilation")) {
        applyOneRingDilation();
    }

    ImGui::SliderFloat(
        "Quality Threshold", &m_qualityThreshold, 0.0f, 1.0f, "%.2f");

    if (ImGui::Button("Select Based on Quality")) {
        selectVerticesBasedOnQuality();
    }

    if (ImGui::Button("Clear Selection")) {
        m_selectionBitMask.assign(m_selectionBitMask.size(), false);
        if (polyscope::hasPointCloud(m_selectedVerticesPointCloudPSID)) {
            polyscope::removePointCloud(m_selectedVerticesPointCloudPSID);
        }
        m_wasSelectionModified = true;
    }

    ImGui::Separator();
}

void VertexSelector::applyOneRingDilation()
{
    // For each selected vertex, identify all its neightbors and select them as
    // well
    std::vector<bool> newSelection = m_selectionBitMask;
    for (int i = 0; i < m_selectionBitMask.size(); ++i) {
        if (m_selectionBitMask[i]) {
            // This vertex is selected, now select its neighbors
            for (int neighborIdx : m_targetMesh.getVertexNeighbors(i)) {
                newSelection[neighborIdx] = true;
            }
        }
    }
    m_selectionBitMask     = newSelection;
    m_wasSelectionModified = true;
}

};  // namespace locremesh