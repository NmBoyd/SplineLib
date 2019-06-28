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
         *  |  d1  u1   0   0   0  |      | x1 |    | b1 |
         *  |  u1  d2   u2  0   0  |      | x2 |    | b2 |
         *  |  0   u2   d3  u3  0  |   *  | x3 |  = | b3 |
         *  |  0   0    u3  d4  u4 |      | x4 |    | b4 |
         *  |  0   0    0   u4  d5 |      | x5 |    | b5 |
         *
         *
         *  The general derivation for this can be found
         *  in Robert Sedgewick's "Algorithms in C++".
         *
         */
        std::array<std::array<Vector3d, NOM_SIZE>, 4> x_col_; // column full of constants to solve for trinomial
        std::array<double, NOM_SIZE> spline_lengths_;

    public:
        Vector3d SplineAtTime(double t);
        double SplineArcLengthIntegrand(int spline, double t);
        double ArcLengthIntegrand(double t);
        double IntegrateSpline(int spline, double t);
        double IntegrateCurve(float t0, float t1);
        Vector3d ConstVelocitySplineAtTime(double t);
        double EvaluateCurveLength();

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