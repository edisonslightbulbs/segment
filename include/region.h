#ifndef REGION_H
#define REGION_H

#include "point.h"
#include <vector>

namespace region {

/**
 * segment
 *   Segments context.
 *
 * @param points
 *   Points to segment.
 *
 * @retval
 *   Segmented points.
 */
std::vector<Point> segment(std::vector<Point>& points);

}
#endif /* REGION_H */
