#ifndef CUBIC_SPLINE_H
#define CUBIC_SPLINE_H
#include "spline.h"

using namespace Eigen;

/** Cubic Spline 
  * Works best with tightly packed waypoint list
  * */
class CubicSpline : public SplineCurve
{
    private:   
        /* The system of linear equations found by solving
         * for the 3 order spline polynomial is given by:
         * A*x = b.  The "x" is represented by x_col and the
         * "b" is represented by b_col in the code.
         *
         * The "A" is formulated with diagonal elements (diag_elems_) and
         * symmetric off-diagonal elements (off_diag_elems_).  The
         * general structure (for six  waypoints) looks like:
         *
         *
         *  |  d1  u1   0   0   0  |      | c1 |    | b1 |
         *  |  u1  d2   u2  0   0  |      | c2 |    | b2 |
         *  |  0   u2   d3  u3  0  |   *  | c3 |  = | b3 |
         *  |  0   0    u3  d4  u4 |      | c4 |    | b4 |
         *  |  0   0    0   u4  d5 |      | c5 |    | b5 |
         *
         *
         *  The general derivation for this can be found
         *  in Robert Sedgewick's "Algorithms in C++".
         *
         */
        std::vector<double> x_col_; // column full of constants to solve for trinomial
        std::vector<double> b_col_;
        std::vector<double> diag_elems_;
        std::vector<double> off_diag_elems_;

    public:
        // Vector3d SplineAtTime(float t);
        // float ArcLengthIntegrand(int splineIndex, float t);
        // float Integrate(int splineIndex, float t);
        // Vector3d ConstVelocitySplineAtTime(float t);
        CubicSpline();
        ~CubicSpline(){}
        /**
         * Evaluate spline for the ith segment for x,y,z params. 
         * The value of param t must be (0<=t<=1)
         */
        std::tuple<Vector3d,Vector3d,Vector3d,double>  Evaluate(int segment, double t) override;

        /* Clear out all the data.
        */
        void ResetDerived() override;
        bool ComputeSpline() override;
        void PrintDerivedData() override;
        bool BuildSpline(std::vector<Vector3d> path, int divisions) override;
        
};

#endif