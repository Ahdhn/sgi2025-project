#pragma once

#include <Eigen/Core>
#include <iostream>

#include "igl/boundary_loop.h"
#include "igl/igl_inline.h"
#include "igl/read_triangle_mesh.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "remesh/src/remesh_botsch.h"

#include "mesh.h"

namespace locremesh {

class VertexSelector
{
   public:
    VertexSelector(Mesh& targetMesh) : m_targetMesh(targetMesh)
    {
        m_selectionBitMask.assign(m_targetMesh.getVertices().rows(), false);
    }

    void applyOneRingDilation();
    void selectVerticesBasedOnQuality();
    void updateSelectedVertices();
    void handleManualVertexSelection(ImGuiIO& io);
    void polyscopeUISection();

    // Get methods -------------------------------------------------------------
    Mesh& getTargetMesh()
    {
        return m_targetMesh;
    }
    const Mesh& getTargetMesh() const
    {
        return m_targetMesh;
    }
    std::string getSelectedVerticesPointCloudPSID() const
    {
        return m_selectedVerticesPointCloudPSID;
    }
    std::string getSelectedVerticesStr() const
    {
        return m_selectedVerticesStr;
    }
    bool wasSelectionModified() const
    {
        return m_wasSelectionModified;
    }
    std::vector<bool>& getSelectedVerticesBitMask()
    {
        return m_selectionBitMask;
    }
    const std::vector<bool>& getSelectedVerticesBitMask() const
    {
        return m_selectionBitMask;
    }

   private:
    Mesh&             m_targetMesh;
    std::string       m_selectedVerticesPointCloudPSID = "selectedVertices";
    std::string       m_selectedVerticesStr;
    bool              m_wasSelectionModified = false;
    std::vector<bool> m_selectionBitMask;
    float             m_qualityThreshold = 0.2;
};

}  // namespace locremesh