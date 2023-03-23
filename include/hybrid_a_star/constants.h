#ifndef HYBRID_A_STAR_CONFIG_H
#define HYBRID_A_STAR_CONFIG_H

#include <cmath>

namespace hybrid_a_star
{
    namespace Constants
    {
        /* data */
        //[#] Max iteration of the algorithm
        static const int maxIter = 30000;
        //[m] Uniformly add pads around the vehicle
        static const double bloating = 0.0;
        //[m] The width of the vehicle
        static const double vehWidth = 0.18 + 2 * bloating;
        //[m] The length of the vehicle
        static const double vehLength = 0.25 + 2 * bloating;
        //[m] The minimum turning radius of the vehicle
        static const double minTurnR = 1;

        //[#] The number of directon
        static const int numDir = 6;

        // //[m] The discretization value of the x and y of the hybrid A* grid
        // static const double deltaXY = 0.5;

        /// [m] --- The number of discretizations in heading
        static const int dimYaw = 72;
        /// [°] --- The discretization value of the heading (goal condition)
        static const double deltaYawDeg = 360 / (double)dimYaw;
        /// [c*M_PI] --- The discretization value of heading (goal condition)
        static const double deltaYawRad = 2 * M_PI / (double)dimYaw;
        /// [c*M_PI] --- The heading part of the goal condition
        static const double deltaYawNegRad = 2 * M_PI - deltaYawRad;

        //[m] The tie breaking parameter
        static const double tieBreaker = 0.1;

        /// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
        static const float factor2D = sqrt(5) / sqrt(2) + 1;
        /// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
        static const float penaltyTurning = 1.05;
        /// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
        static const float penaltyReversing = 2.0;
        /// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
        static const float penaltyCOD = 2.0;
        /// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
        static const float dubinsShotDistance = 100;
        /// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
        static const float dubinsStepSize = 1;
        /// [#] --- The period of the RS shot
        static const int rsShotPeriod = 10;

    };

}

#endif
