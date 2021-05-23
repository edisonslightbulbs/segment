#include <vector>

#include "outliers.h"
#include "proposal.h"
#include "region.h"
#include "svd.h"

std::vector<Point> region::segment(std::vector<Point>& points)
{
    /** remove outliers */
    std::vector<Point> denoisedPcl = outliers::filter(points);

    /** compute svd */
    std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf> USV;
    USV = svd::compute(denoisedPcl);

    /** extract coarse segment */
    std::vector<Point> coarseSeg = proposal::grow(USV, denoisedPcl);

    /** extract final segment */
    std::vector<Point> finalSeg = outliers::filter(coarseSeg);

    /** return interaction context */
    return finalSeg;
}

std::pair<Point, Point> region::queryBoundary(std::vector<Point>& points)
{
    std::vector<int16_t> X(points.size());
    std::vector<int16_t> Y(points.size());
    std::vector<int16_t> Z(points.size());
    for (auto& point : points) {
        X.push_back(point.m_xyz[0]);
        Y.push_back(point.m_xyz[1]);
        Z.push_back(point.m_xyz[2]);
    }

    /** find boundary given a set of points */
    int16_t xMax = (int16_t)*std::max_element(X.begin(), X.end());
    int16_t xMin = (int16_t)*std::min_element(X.begin(), X.end());
    int16_t yMax = (int16_t)*std::max_element(Y.begin(), Y.end());
    int16_t yMin = (int16_t)*std::min_element(Y.begin(), Y.end());
    int16_t zMax = (int16_t)*std::max_element(Z.begin(), Z.end());
    int16_t zMin = (int16_t)*std::min_element(Z.begin(), Z.end());

    Point min(xMin, yMin, zMin);
    Point max(xMax, yMax, zMax);
    return { min, max };
}
