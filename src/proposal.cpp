#include <cmath>

#include "edge.h"
#include "io.h"
#include "proposal.h"

#define V3_ARGMIN 350
#define U_ARGMIN 3
const int C0 = 0; // column 0
const int C1 = 1; // column 1
const int C2 = 2; // column 2

std::vector<Point> proposal::grow(SVD& svd, std::vector<Point>& points)
{
    // get unitary matrix normals
    Eigen::MatrixXf x = svd.m_usv.matrixU().col(0);
    Eigen::MatrixXf y = svd.m_usv.matrixU().col(1);
    Eigen::MatrixXf z = svd.m_usv.matrixU().col(2);

    // determine region of interest
    // todo: optimize:
    std::vector<Point> roi;
    Eigen::Vector3d vec;
    Eigen::Vector3d svdUNormal;
    Eigen::Vector3d svdV3Normal = svd.getV3Normal();

    int index = 0;
    for (const auto& point : points) {
        vec = Eigen::Vector3d(svd.m_vectors(index, C0),
            svd.m_vectors(index, C1), svd.m_vectors(index, C2));
        svdUNormal = Eigen::Vector3d(x(index, C0), y(index, C0), z(index, C0));

        if (std::abs(svdV3Normal.dot(vec)) < V3_ARGMIN) {
            if (svdUNormal.dot(vec) < U_ARGMIN) {
                if (edge::detect(z(index, C0))) {
                    roi.push_back(point);
                }
            }
        }
        index++;
    }

    // clean coarse segment
    // todo optimize:
    const int16_t CLEAN = 100;
    Point centroid = Point::centroid(roi);
    std::vector<Point> segment;
    for (auto& point : roi) {
        if (point.m_xyz[2] < centroid.m_xyz[2] + CLEAN) {
            segment.push_back(point);
        }
    }
    return segment;
}
