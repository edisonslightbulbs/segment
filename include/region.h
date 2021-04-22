#ifndef REGION_H
#define REGION_H

#include <Eigen/Dense>
#include <vector>

#include "point.h"
#include "outliers.h"
#include "svd.h"
#include "proposal.h"

namespace region{

std::vector<Point> segment(std::vector<Point>& points)
{
    /** filter out outliers */
    std::vector<Point> denoisedPcl = outliers::filter(points);

    /** compute svd */
    std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf> USV;
    USV = svd::compute(denoisedPcl);

    /** coarse segment */
    std::vector<Point> coarseSeg = proposal::grow(USV, denoisedPcl);

    /** final segment */
    std::vector<Point> finalSeg = outliers::filter(coarseSeg);

    /** return interaction context */
    return finalSeg;
}

    std::pair<Point, Point> queryBoundary(
            std::vector<Point>& context)
    {
        std::vector<float> X(context.size());
        std::vector<float> Y(context.size());
        std::vector<float> Z(context.size());
        for (auto& point : context) {
            X.push_back(point.m_xyz[0]);
            Y.push_back(point.m_xyz[1]);
            Z.push_back(point.m_xyz[2]);
        }

        /** find boundary given a set of points */
        float xMax = *std::max_element(X.begin(), X.end());
        float xMin = *std::min_element(X.begin(), X.end());
        float yMax = *std::max_element(Y.begin(), Y.end());
        float yMin = *std::min_element(Y.begin(), Y.end());
        float zMax = *std::max_element(Z.begin(), Z.end());
        float zMin = *std::min_element(Z.begin(), Z.end());

        Point min(xMin, yMin, zMin);
        Point max(xMax, yMax, zMax);
        return { min, max };
    }


}
#endif /* REGION_H */
