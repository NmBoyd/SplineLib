#include "../include/cubic_spline.h"

CubicSpline::CubicSpline()
{
    
}


std::tuple<Vector3d,Vector3d,Vector3d,double>  CubicSpline::Evaluate(int segment, double t)
{
    const std::vector<Vector3d>& points = GetPoints();
    // assert(t >= 0);
    // assert(t <= 1.0);
    // assert(segment >= 0);
    // assert(segment < (points.size()));

    Vector3d spline_positions = x_col_[segment][0] + x_col_[segment][1]*t + x_col_[segment][2]*t*t + x_col_[segment][3]*t*t*t;
    Vector3d spline_velocity = x_col_[segment][1] + x_col_[segment][2]*2*t + x_col_[segment][3]*3*t*t;
    
    Vector3d spline_acceleration = x_col_[segment][2]*2 + x_col_[segment][3]*6*t;
   
    // TODO: Should be more unsteady
    double   spline_curvature = (spline_velocity.x()*spline_acceleration.y())-(spline_velocity.y()*spline_acceleration.x())/
                                pow((spline_velocity.x()*spline_velocity.x()+spline_velocity.y()*spline_velocity.y()),(3/2));
    
    return std::make_tuple( spline_positions, 
                            spline_velocity,
                            spline_acceleration,
                            spline_curvature );
}

void CubicSpline::ResetDerived()
{
    // for(int i=0;i<4;i++)
    //     x_col_[i].clear();
    // setpoints_.clear();
}

bool CubicSpline::ComputeSpline()
{
    const std::vector<Vector3d> p = GetPoints();

    int n = p.size()-1;

    Vector3d a[p.size()];
    for (int i = 1; i < n; i++)
        a[i] = 3*((p[i+1] - 2*p[i] + p[i-1]));

    float l[p.size()];
    float mu[p.size()];
    Vector3d z[p.size()];

    l[0] = l[n] = 1;
    mu[0] = 0;
    z[0] = z[n] = Vector3d(0.0, 0.0, 0.0);
    x_col_[n][2] = Vector3d(0.0, 0.0, 0.0);

    for (int i = 1; i <= n-1; i++)
    {
        l[i] = 4 - mu[i-1];
        mu[i] = 1 / l[i];
        z[i] = (a[i] - z[i-1]) / l[i];
    }

    for (int i = 0; i < p.size(); i++)
        x_col_[i][0] = p[i];    // set all the "a" variables to the initial position
     

    for (int j = n-1; j >= 0; j--)
    {
        x_col_[j][2] = z[j] - mu[j] * x_col_[j+1][2];
        x_col_[j][3] = (1.0f / 3.0f)*(x_col_[j+1][2] - x_col_[j][2]);
        x_col_[j][1] = p[j + 1] - p[j] - x_col_[j][2] - x_col_[j][3];
    }
    
    // Solved for the x column coefficients (a,b,c,d in polynomial)
    return true;
}

// float ArcLengthIntegrand(int spline, float t)
// {
//     float tt = t*t;

//     Vector3d dv = m_coeffs[spline][1] + 2 * m_coeffs[spline][2] * t + 3 * m_coeffs[spline][3] * tt;
//     float xx = dv.x*dv.x;
//     float yy = dv.y*dv.y;
//     float zz = dv.z*dv.z;

//     return sqrt(xx + yy + zz);
// }

// // Composite Simpson's Rule, Burden & Faires - Numerical Analysis 9th, algorithm 4.1
// float Integrate(int spline, float t)
// {
//     int n = 16;
//     float h = t / n;
//     float XI0 = ArcLengthIntegrand(spline, t);
//     float XI1 = 0;
//     float XI2 = 0;

//     for (int i = 0; i < n; i++)
//     {
//         float X = i*h;
//         if (i % 2 == 0)
//             XI2 += ArcLengthIntegrand(spline, X);
//         else
//             XI1 += ArcLengthIntegrand(spline, X);
//     }

//     float XI = h * (XI0 + 2 * XI2 + 4 * XI1) * (1.0f / 3);
//     return XI;
// }

// Vector3d ConstVelocitySplineAtTime(float t)
// {ze();++i)
    // {
    //     c_pathx[i] = c_spline.GetPositionProfile()[i].x();
    //     c_pathy[i] = c_spline.GetPositionProfile()[i].y();
    //     c_pathvx[i] = c_spline.GetVelocityProfile()[i].x();
    //     c_pathvy[i] = c_spline.GetVelocityProfile()[i].y();
    //     c_pathax[i] = c_spline.GetAccelerationProfile()[i].x();
    //     c_pathay[i] = c_spline.GetAccelerationProfile()[i].y();
    // }
//     int spline = 0;ze();++i)
    // {
    //     c_pathx[i] = c_spline.GetPositionProfile()[i].x();
    //     c_pathy[i] = c_spline.GetPositionProfile()[i].y();
    //     c_pathvx[i] = c_spline.GetVelocityProfile()[i].x();
    //     c_pathvy[i] = c_spline.GetVelocityProfile()[i].y();
    //     c_pathax[i] = c_spline.GetAccelerationProfile()[i].x();
    //     c_pathay[i] = c_spline.GetAccelerationProfile()[i].y();
    // }
//     while (t > m_lengths[ze();++i)
    // {
    //     c_pathx[i] = c_spline.GetPositionProfile()[i].x();
    //     c_pathy[i] = c_spline.GetPositionProfile()[i].y();
    //     c_pathvx[i] = c_spline.GetVelocityProfile()[i].x();
    //     c_pathvy[i] = c_spline.GetVelocityProfile()[i].y();
    //     c_pathax[i] = c_spline.GetAccelerationProfile()[i].x();
    //     c_pathay[i] = c_spline.GetAccelerationProfile()[i].y();
    // }spline])
//     {
//         t -= m_lengths[spline];
//         spline += 1;
//     }

//     float s = t / m_lengths[spline]; // Here's our initial guess.

//     // Do some Newton-Rhapsons.
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);
//     s = s - (Integrate(spline, s) - t) / ArcLengthIntegrand(spline, s);

//     return SplineAtTime(spline + s);
// }

void CubicSpline::PrintDerivedData()
{
    std::cout << " Control Points " << std::endl;
}

bool CubicSpline::BuildSpline(std::vector<Vector3d> setpoints, int divisions)
{
    assert(setpoints.size() > 2);
    
    for(int idx = 0; idx<setpoints.size(); idx++)
    {
        AddPoint(setpoints[idx]);
    }
    // Smooth them.
    ComputeSpline();
    
    // Loop through all segments and create the overall spline
    for(int idx = 0; idx < GetPoints().size()-1; idx++)
    {
        for(int division = 0; division <= divisions; division++)
        {
            double t = division*1.0/divisions;
            std::tuple<Vector3d, Vector3d, Vector3d, double> state_info = Evaluate(idx, t);
            pos_profile_.push_back(std::get<0>(state_info));    // this is backwards
            vel_profile_.push_back(std::get<1>(state_info));
            accel_profile_.push_back(std::get<2>(state_info));
            curvature_profile_.push_back(std::get<3>(state_info));
        }
    }

    return true;
}