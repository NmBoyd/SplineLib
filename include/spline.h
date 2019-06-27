#ifndef SPLINE_H
#define SPLINE_H

//internals
#include <iostream>
#include <vector>
#include <math.h>
#include <assert.h>
// externals 
#include <eigen3/Eigen/Dense>

using namespace Eigen;
class Spline
{
    private:
        std::vector<Vector3d> points_;  // array of points in plot
        bool elim_colinear_pts_;    // removes colinear points if enabled

    protected:
        /* Override */
        virtual void ResetDerived() = 0;
        enum
        {
            NOM_SIZE = 32
        };

    public:
         
        
        Spline()
        {
            points_.reserve(NOM_SIZE);
            elim_colinear_pts_ = false ;
        }
        ~Spline(){}

        const std::vector<Vector3d>& GetPoints() { return points_; }
        bool GetElimColinearPoints() { return elim_colinear_pts_; }
        void SetelimColinearPoints(bool elim) { elim_colinear_pts_ = true;}

        /* Virtual Spline Properties */
        virtual Vector3d Evaluate(int seg, double t) = 0;
        virtual std::vector<Vector3d> BuildSpline(std::vector<Vector3d> path, int divisions)=0;
        virtual bool ComputeSpline() = 0;
        virtual void PrintDerivedData() {}

        void Reset();
        void AddPoint(const Vector3d& pt);
        void PrintData(int segments);

};

#endif 
