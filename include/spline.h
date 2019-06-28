#ifndef SPLINE_H
#define SPLINE_H

//internals
#include <iostream>
#include <tuple>
#include <vector>
#include <math.h>
#include <assert.h>
// externals 
#include <eigen3/Eigen/Dense>

using namespace Eigen;
class SplineCurve
{
    private:
        std::vector<Vector3d> points_;              // array of primary setpoints added to plot
        bool elim_colinear_pts_;                    // removes colinear points if enabled
        
    protected:

        std::vector<Vector3d> pos_profile_;         // Curve of all positions 
        std::vector<Vector3d> vel_profile_;         // Curve velocity profile
        std::vector<Vector3d> accel_profile_;       // Curve acceleration profile
        std::vector<double>   curvature_profile_;   // Curvature (k) of spline between knots. 
                                                    // Each curvature index represents a segment
        /* Override */
        virtual void ResetDerived() = 0;
        enum
        {
            NOM_SIZE = 32
        };

    public:
         
        
        SplineCurve()
        {
            points_.reserve(NOM_SIZE);
            elim_colinear_pts_ = false ;
        }
        ~SplineCurve(){}

        const std::vector<Vector3d>& GetPoints() { return points_; }
        const std::vector<Vector3d>& GetPositionProfile() { return pos_profile_; }
        const std::vector<Vector3d>& GetVelocityProfile() { return vel_profile_; }
        const std::vector<Vector3d>& GetAccelerationProfile() { return accel_profile_; }
        const std::vector<double>&   GetCurvatureProfile() { return curvature_profile_; }
       
        bool GetElimColinearPoints() { return elim_colinear_pts_; }
        void SetelimColinearPoints(bool elim) { elim_colinear_pts_ = true;}

        /* Virtual Spline Properties */
        // A segment is the curve between two setpoints
        virtual std::tuple<Vector3d,Vector3d,Vector3d,double>  Evaluate(int seg, double t) = 0;
        virtual bool BuildSpline(std::vector<Vector3d> setpoints, int divisions)=0;
        virtual bool ComputeSpline() = 0;

        // virtual double ArcLengthIntegrand(int spline, double t) = 0;
        // virtual double Integrate(int spline, double t) = 0;
        // virtual Vector3d ConstVelocitySplineAtTime(double t) = 0;
        virtual void PrintDerivedData() {}

        void Reset();
        void AddPoint(const Vector3d& pt);
        void PrintData(int segments);

};

#endif 
