#pragma once

#include <Eigen/Core>
#include <exception>
#include <iostream>

#include "indicatorFunctions.h"
#include "stb_image.h"
#include "utils.h"

namespace locremesh {

class Mesh
{
   public:
    Mesh() = default;

    Mesh(std::string meshFilename,
         std::string textureFilename,
         std::string polyscopeID = "mesh")
        : m_polyscopeID(polyscopeID)
    {
        if (!igl::read_triangle_mesh(meshFilename, m_vertices, m_faces)) {
            throw std::runtime_error("Could not load mesh from " +
                                     meshFilename);
        }
        if (!textureFilename.empty()) {
            loadTexture(textureFilename);
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
          m_boundaryBitMask(other.m_boundaryBitMask),
          m_textureWidth(other.m_textureWidth),
          m_textureHeight(other.m_textureHeight),
          m_textureChannels(other.m_textureChannels),
          m_textureColor(other.m_textureColor)
    {
    }

    void loadTexture(std::string textureFilename);
    void calculateMeshQuality();
    void calculateUVParametrization(bool useCurrentUV = true);
    void identifyBoundaryVertices();
    void updateVertexPositions(Eigen::MatrixXd& newVertices);

    polyscope::SurfaceMesh* polyscopeRegisterSurfaceMesh();
    std::set<int>           getVertexNeighbors(int vertexIdx);

    // Get methods
    // -------------------------------------------------------------
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
    void setParamatrizationIterations(int iterations)
    {
        m_parametrizationIterations = iterations;
    }


   private:
    Eigen::MatrixXd   m_vertices;
    Eigen::MatrixXi   m_faces;
    Eigen::VectorXd   m_quality;
    Eigen::MatrixXd   m_uvCoords;
    std::vector<bool> m_boundaryBitMask;

    // Polyscope
    std::string m_polyscopeID;

    // Parametrization
    int m_parametrizationIterations = 10;

    // Texture
    int m_textureWidth;
    int m_textureHeight;
    int m_textureChannels;
    std::vector<std::array<float, 3>>
        m_textureColor;  // The actual texture pixel data
};

}  // namespace locremesh