#pragma once

#include <Eigen/Core>
#include <set>
#include <vector>

#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "remesh/src/remesh_botsch.h"

#include "mesh.h"
#include "vertexSelector.h"

namespace locremesh {

class BotschRemesher
{
   public:
    BotschRemesher(VertexSelector& vertexSelector,
                   float           initialTargetEdgeLength,
                   int             initialIterations,
                   bool            initialShouldProject)
        : m_vertexSelector(vertexSelector),
          m_targetEdgeLength(initialTargetEdgeLength),
          m_iterations(initialIterations),
          m_shouldProject(initialShouldProject),
          m_resultingMesh(vertexSelector.getTargetMesh())
    {
    }

    void remesh(std::string resultingMeshPolyscopeID = "botschRemeshed");
    void polyscopeUISection();

   public:
    VertexSelector& m_vertexSelector;
    Mesh&           m_resultingMesh;
    bool            m_keepOriginalMesh = false;
    float           m_targetEdgeLength;
    int             m_iterations;
    bool            m_shouldProject;
};

};  // namespace locremesh