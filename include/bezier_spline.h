#include "spline.h"

/* Bezier Spline Implementation
 * Based on this article:
 * http://www.particleincell.com/blog/2012/bezier-splines/
 */
class BezierSpline : public Spline
{
    private:
    std::vector<Vector3d> _p1Points;
    std::vector<Vector3d> _p2Points;
    public:
    BezierSpline()
    {
        _p1Points.reserve(NOM_SIZE);
        _p2Points.reserve(NOM_SIZE);
    }

    /* Evaluate the spline for the ith segment
        * for parameter.  The value of parameter t must
        * be between 0 and 1.
        */
    Vector3d Evaluate(int seg, double t) override;
    void ResetDerived() override;
    bool ComputeSpline() override;
    void PrintDerivedData() override;
    std::vector<Vector3d> BuildSpline(std::vector<Vector3d> path, int divisions) override;
};