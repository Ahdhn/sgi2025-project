#include "mesh.h"
#include "param.h"

namespace locremesh {

void Mesh::loadTexture(std::string textureFilename)
{
    if (!textureFilename.empty()) {
        // Force 3 channels (RGB) for simplicity
        int            desired_channels = 3;
        unsigned char* data             = stbi_load(textureFilename.c_str(),
                                        &m_textureWidth,
                                        &m_textureHeight,
                                        &m_textureChannels,
                                        desired_channels);
        m_textureChannels               = desired_channels;

        if (!data) {
            std::cout << "failed to load " << textureFilename << std::endl;
            return;
        }

        m_textureColor.resize(m_textureWidth * m_textureHeight);

        for (int j = 0; j < m_textureHeight; j++) {
            for (int i = 0; i < m_textureWidth; i++) {
                int pix_ind = (j * m_textureWidth + i) * m_textureChannels;

                m_textureColor[j * m_textureWidth + i] = {
                    data[pix_ind + 0] / 255.f,
                    data[pix_ind + 1] / 255.f,
                    data[pix_ind + 2] / 255.f};
            }
        }
        stbi_image_free(data);
    }
}

void Mesh::updateVertexPositions(Eigen::MatrixXd& newVertices)
{
    assert(newVertices.rows() == m_vertices.rows() &&
           newVertices.cols() == m_vertices.cols());
    m_vertices = newVertices;
}

polyscope::SurfaceMesh* Mesh::polyscopeRegisterSurfaceMesh()
{
    assert(m_quality.size() == m_faces.rows());  // Quality has been calculated
    assert(m_uvCoords.rows() == m_vertices.rows() && m_uvCoords.cols() == 2);

    if (polyscope::hasSurfaceMesh(m_polyscopeID)) {
        polyscope::removeSurfaceMesh(m_polyscopeID);
    }

    auto psSurfaceMesh =
        polyscope::registerSurfaceMesh(m_polyscopeID, m_vertices, m_faces);

    // Add Triangle Quality
    auto psFaceScalar =
        psSurfaceMesh->addFaceScalarQuantity("Triangle Quality", m_quality);
    psFaceScalar->setMapRange(std::make_pair<int, int>(0, 1));

    // Add Parametrization
    auto psVertexParam =
        psSurfaceMesh->addVertexParameterizationQuantity("UV Map", m_uvCoords);

    // Add texture
    if (!m_textureColor.empty()) {
        auto psTextureColor = psSurfaceMesh->addTextureColorQuantity(
            "Texture",
            *psVertexParam,
            m_textureWidth,
            m_textureHeight,
            m_textureColor,
            polyscope::ImageOrigin::LowerLeft);
        psTextureColor->setEnabled(true);
    }

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

void Mesh::calculateUVParametrization(bool useCurrentUV)
{
    std::cout << "Calculating UV parametrization..." << std::endl;
    if (useCurrentUV) {
        m_uvCoords = param<double>(
            m_vertices, m_faces, m_uvCoords, m_parametrizationIterations);
    } else {
        Eigen::MatrixXd emptyUV;
        m_uvCoords = param<double>(
            m_vertices, m_faces, emptyUV, m_parametrizationIterations);
    }

    // Normalize UV coordinates to [0,1] range and flip V-coordinate.
    // This is necessary because the parameterization function doesn't
    // guarantee the output is in the [0, 1] range, and we need to match the
    // image coordinate system (stb_image loads top-to-bottom).
    Eigen::Vector2d uv_min   = m_uvCoords.colwise().minCoeff();
    Eigen::Vector2d uv_max   = m_uvCoords.colwise().maxCoeff();
    Eigen::Vector2d uv_range = uv_max - uv_min;

    // Avoid division by zero for degenerate UV maps (e.g., a line).
    if (uv_range.x() < 1e-10)
        uv_range.x() = 1.0;
    if (uv_range.y() < 1e-10)
        uv_range.y() = 1.0;

    // Apply normalization
    m_uvCoords.col(0) = (m_uvCoords.col(0).array() - uv_min.x()) / uv_range.x();
    m_uvCoords.col(1) = (m_uvCoords.col(1).array() - uv_min.y()) / uv_range.y();

    std::cout << "Finished calculating UV parametrization" << std::endl;
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