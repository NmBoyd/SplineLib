#include "../include/cubic_spline.h"

CubicSpline::CubicSpline()
{
    x_col_.reserve(NOM_SIZE);
    b_col_.reserve(NOM_SIZE);
    diag_elems_.reserve(NOM_SIZE);
    off_diag_elems_.reserve(NOM_SIZE);
}


std::tuple<Vector3d,Vector3d,Vector3d,double>  CubicSpline::Evaluate(int segment, double t)
{
    const std::vector<Vector3d>& points = GetPoints();
    // assert(t >= 0);
    // assert(t <= 1.0);
    // assert(segment >= 0);
    // assert(segment < (points.size()));

    const double ONE_SIXTH = 1.0/6.0;
    double oneMinust = 1.0-t;
    double t3Minust = t*t*t-t;
    double oneMinust3minust = oneMinust*oneMinust*oneMinust-oneMinust;
    double deltaX = points[segment+1].x()-points[segment].x();
    double yValue = t*points[segment+1].y() + 
                    oneMinust*points[segment].y() +
                    ONE_SIXTH*deltaX*deltaX*(t3Minust*x_col_[segment+1]-oneMinust3minust*x_col_[segment]);
    double xValue = t*(points[segment+1].x()-points[segment].x())+points[segment].x();
    double zValue = 0;

    return std::make_tuple( Vector3d(xValue,yValue,zValue), 
                            Vector3d(0.0,0.0,0.0),
                            Vector3d(0.0,0.0,0.0),
                            0.0 );
}

void CubicSpline::ResetDerived()
{
    diag_elems_.clear();
    x_col_.clear();
    b_col_.clear();
    off_diag_elems_.clear();
}

bool CubicSpline::ComputeSpline()
{
    const std::vector<Vector3d> p = GetPoints();

    b_col_.resize(p.size());
    x_col_.resize(p.size());
    diag_elems_.resize(p.size());
    
    for(int idx = 1;idx < p.size(); ++idx)
    {
        diag_elems_[idx] = 2*(p[idx+1].x()-p[idx-1].x());
    }
    for(int idx = 0; idx < p.size(); ++idx)
    {
        off_diag_elems_[idx] = p[idx+1].x()-p[idx].x();
    }
    for(int idx = 1; idx < p.size(); ++idx)
    {
        b_col_[idx] = 6.0*((p[idx+1].y()-p[idx].y())/off_diag_elems_[idx] - 
                            (p[idx].y()-p[idx-1].y())/off_diag_elems_[idx-1]);
    }

    x_col_[0] = 0.0;
    x_col_[p.size()-1] = 0.0;
    for(int idx = 1; idx < p.size()-1; ++idx)
    {
        b_col_[idx+1] = b_col_[idx+1] - b_col_[idx]*off_diag_elems_[idx]/diag_elems_[idx];
        diag_elems_[idx+1] = diag_elems_[idx+1] - off_diag_elems_[idx]*off_diag_elems_[idx];
    }
    for(int idx = (int)p.size()-2; idx > 0; --idx)
    {
        x_col_[idx] = (b_col_[idx] - off_diag_elems_[idx]*x_col_[idx+1])/diag_elems_[idx];
    }
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
// {
//     int spline = 0;
//     while (t > m_lengths[spline])
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
        }
    }

    return true;
}