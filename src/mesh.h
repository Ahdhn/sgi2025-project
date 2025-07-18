#pragma once

#include <Eigen/Core>
#include <exception>
#include <iostream>

#include "indicatorFunctions.h"
#include "utils.h"

namespace locremesh {
class Mesh
{
   public:
    Mesh() = default;

    Mesh(std::string filename, std::string polyscopeID = "mesh")
        : m_polyscopeID(polyscopeID)
    {
        if (!igl::read_triangle_mesh(filename, m_vertices, m_faces)) {
            throw std::runtime_error("Could not load mesh from " + filename);
        }
        calculateMeshQuality();
        calculateUVParametrization();
        identifyBoundaryVertices();
    }

    Mesh(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces)
        : m_vertices(vertices), m_faces(faces)
    {
        calculateMeshQuality();
        calculateUVParametrization();
        identifyBoundaryVertices();
    }

    Mesh(const Mesh& other)
        : m_polyscopeID(other.m_polyscopeID),
          m_vertices(other.m_vertices),
          m_faces(other.m_faces),
          m_quality(other.m_quality),
          m_uvCoords(other.m_uvCoords),
          m_boundaryBitMask(other.m_boundaryBitMask)
    {
    }


    polyscope::SurfaceMesh* polyscopeRegisterSurfaceMesh();
    void                    calculateMeshQuality();
    void                    calculateUVParametrization();
    void                    identifyBoundaryVertices();
    std::set<int>           getVertexNeighbors(int vertexIdx);

    // Get methods -------------------------------------------------------------
    const int getVertexCount() const
    {
        return m_vertices.rows();
    }
    const int getFaceCount() const
    {
        return m_faces.rows();
    }
    const Eigen::MatrixXd& getVertices() const
    {
        return m_vertices;
    }
    const Eigen::MatrixXi& getFaces() const
    {
        return m_faces;
    }
    const Eigen::VectorXd& getQuality() const
    {
        return m_quality;
    }
    const std::vector<bool>& getBoundaryBitMask() const
    {
        return m_boundaryBitMask;
    }
    Eigen::MatrixXd& getVertices()
    {
        return m_vertices;
    }
    Eigen::MatrixXi& getFaces()
    {
        return m_faces;
    }
    Eigen::VectorXd& getQuality()
    {
        return m_quality;
    }
    std::vector<bool>& getBoundaryBitMask()
    {
        return m_boundaryBitMask;
    }
    std::string getPolyscopeID() const
    {
        return m_polyscopeID;
    }
    void setPolyscopeID(std::string polyscopeID)
    {
        m_polyscopeID = polyscopeID;
    }


   private:
    std::string       m_polyscopeID;
    Eigen::MatrixXd   m_vertices;
    Eigen::MatrixXi   m_faces;
    Eigen::VectorXd   m_quality;
    Eigen::MatrixXd   m_uvCoords;
    std::vector<bool> m_boundaryBitMask;
};
}  // namespace locremesh