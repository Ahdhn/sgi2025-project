#pragma once
#include <Eigen/Core>

#include "igl/read_triangle_mesh.h"

#include "polyscope/surface_mesh.h"

#include "polyscope/texture_map_quantity.h"

#include "param.h"

#include "stb_image.h"

void add_image()
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    std::string mesh_path("../cloth.obj");
    std::string img_path("../apple.png");

    igl::read_triangle_mesh(mesh_path, V, F);

    Eigen::Matrix<double, Eigen::Dynamic, 2> UV = param<double>(V, F);

    // Normalize UV coordinates to [0,1] range if needed
    if (UV.minCoeff() < 0.0 || UV.maxCoeff() > 1.0) {
        Eigen::Vector2d uv_min   = UV.colwise().minCoeff();
        Eigen::Vector2d uv_max   = UV.colwise().maxCoeff();
        Eigen::Vector2d uv_range = uv_max - uv_min;

        // Avoid division by zero
        if (uv_range.x() < 1e-10)
            uv_range.x() = 1.0;
        if (uv_range.y() < 1e-10)
            uv_range.y() = 1.0;

        for (int i = 0; i < UV.rows(); i++) {
            UV(i, 0) = (UV(i, 0) - uv_min.x()) / uv_range.x();
            UV(i, 1) = (UV(i, 1) - uv_min.y()) / uv_range.y();
        }
    }

    int width, height, channels;

    unsigned char* data =
        stbi_load(img_path.c_str(), &width, &height, &channels, 0);

    if (!data) {
        std::cout << "failed to load " << img_path << std::endl;
        return;
    }

    std::vector<float> image_scalar(width * height);

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {

            int pix_ind = (j * width + i) * channels;

            unsigned char p_r = data[pix_ind + 0];
            unsigned char p_g = data[pix_ind + 1];
            unsigned char p_b = data[pix_ind + 2];

            std::array<float, 3> val{p_r / 255.f, p_g / 255.f, p_b / 255.f};


            image_scalar[j * width + i] = (val[0] + val[1] + val[2]) / 3.;
        }
    }


    polyscope::init();
    auto* m = polyscope::registerSurfaceMesh("mesh", V, F);

    auto qParam = m->addVertexParameterizationQuantity("param", UV);

    polyscope::SurfaceTextureScalarQuantity* qScalar =
        m->addTextureScalarQuantity("tScalar",
                                    *qParam,
                                    width,
                                    height,
                                    image_scalar,
                                    polyscope::ImageOrigin::UpperLeft);
    qScalar->setFilterMode(polyscope::FilterMode::Nearest);
    qScalar->setEnabled(true);

    stbi_image_free(data);

    polyscope::show();
}