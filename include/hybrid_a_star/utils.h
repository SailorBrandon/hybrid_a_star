#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <algorithm>

namespace hybrid_a_star
{

    namespace Utils
    {

        static inline double normalizeYawRad(double yaw)
        {
            while (yaw > M_PI)
            {
                yaw -= 2 * M_PI;
            }
            while (yaw < -M_PI)
            {
                yaw += 2 * M_PI;
            }
            return yaw;
        }

        static inline double normalizeYawDeg(double yaw)
        {
            while (yaw > 180)
            {
                yaw -= 360;
            }
            while (yaw < -180)
            {
                yaw += 360;
            }
            return yaw;
        }

        static inline double toRad(double deg)
        {
            return normalizeYawRad(deg * M_PI / 180);
        }

        static inline double clip(double n, double lower, double upper)
        {
            return std::max(lower, std::min(n, upper));
        }

    }
}
#endif