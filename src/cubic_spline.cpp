#include "../include/cubic_spline.h"

CubicSpline::CubicSpline()
{
    x_col_.reserve(NOM_SIZE);
    b_col_.reserve(NOM_SIZE);
    diag_elems_.reserve(NOM_SIZE);
    off_diag_elems_.reserve(NOM_SIZE);
}


Vector3d CubicSpline::Evaluate(int segment, double t)
{
    const std::vector<Vector3d>& points = GetPoints();
    assert(t >= 0);
    assert(t <= 1.0);
    assert(segment >= 0);
    assert(segment < (points.size()));

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
    return Vector3d(xValue,yValue,zValue);
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

void CubicSpline::PrintDerivedData()
{
    std::cout << " Control Points " << std::endl;\
}

std::vector<Vector3d> CubicSpline::BuildSpline(std::vector<Vector3d> path, int divisions)
{
    assert(path.size() > 2);

   
    for(int idx = 0; idx<path.size(); idx++)
    {
        AddPoint(path[idx]);
    }
    path.clear();
    // Smooth them.
    ComputeSpline();
    
    // Push them back in.
    for(int idx = GetPoints().size()-1; idx >= 0; --idx)
    {
        for(int division = divisions-1; division >= 0; --division)
        {
            double t = division*1.0/divisions;
            path.push_back(Evaluate(idx, t));
        }
    }

    return path;
}