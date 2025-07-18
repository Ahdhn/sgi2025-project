#include "mesh.h"
#include "param.h"

namespace locremesh {

polyscope::SurfaceMesh* Mesh::polyscopeRegisterSurfaceMesh()
{
    assert(m_quality.size() == m_faces.rows());  // Quality has been calculated
    assert(m_uvCoords.rows() == m_vertices.rows() && m_uvCoords.cols() == 2);

    if (polyscope::hasSurfaceMesh(m_polyscopeID)) {
        polyscope::removeSurfaceMesh(m_polyscopeID);
    }

    auto psSurfaceMesh =
        polyscope::registerSurfaceMesh(m_polyscopeID, m_vertices, m_faces);
    psSurfaceMesh->setEdgeWidth(0.8f);

    // Add Parametrization
    auto psVertexParam =
        psSurfaceMesh->addVertexParameterizationQuantity("UV Map", m_uvCoords);
    psVertexParam->setEnabled(true);

    // Add Triangle Quality
    auto psFaceScalar =
        psSurfaceMesh->addFaceScalarQuantity("Quality", m_quality);
    psFaceScalar->setMapRange(std::make_pair<int, int>(0, 1));

    return psSurfaceMesh;
}

void Mesh::identifyBoundaryVertices()
{
    m_boundaryBitMask.assign(m_vertices.rows(), false);
    for (auto loop : getMeshBoundaryLoops(m_vertices, m_faces)) {
        for (auto idx : loop) {
            m_boundaryBitMask[idx] = true;
        }
    }
}

void Mesh::calculateUVParametrization()
{
    m_uvCoords = param<double>(m_vertices, m_faces);
}

void Mesh::calculateMeshQuality()
{
    m_quality = indFuncTriangleQuality(m_vertices, m_faces);
}

std::set<int> Mesh::getVertexNeighbors(int vertexIdx)
{
    std::set<int> neighbors;
    for (int i = 0; i < m_faces.rows(); ++i) {
        for (int j = 0; j < m_faces.cols(); ++j) {
            if (m_faces(i, j) == vertexIdx) {
                // Found the vertex in this face, add its neighbors in the face
                neighbors.insert(m_faces(i, (j + 1) % 3));
                neighbors.insert(m_faces(i, (j + 2) % 3));
            }
        }
    }
    return neighbors;
}

};  // namespace locremesh