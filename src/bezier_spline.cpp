
#include "../include/bezier_spline.h"
//http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf
Vector3d BezierSpline::Evaluate(int seg, double t)
{
//   assert(seg < _p1Points.size());
//   assert(seg < _p2Points.size());

    double omt = 1.0 - t;

    Vector3d p0(GetPoints()[seg]);
    Vector3d p1(_p1Points[seg]);
    Vector3d p2(_p2Points[seg]);
    Vector3d p3(GetPoints()[seg+1]);

    double xVal = omt*omt*omt*p0.x() + 3*omt*omt*t*p1.x() +3*omt*t*t*p2.x()+t*t*t*p3.x();
    double yVal = omt*omt*omt*p0.y() + 3*omt*omt*t*p1.y() +3*omt*t*t*p2.y()+t*t*t*p3.y();
    double zVal = 0;
    return Vector3d(xVal,yVal,zVal);
}

/* Clear out all the data.
*/
void BezierSpline::ResetDerived()
{
    _p1Points.clear();
    _p2Points.clear();
}


bool BezierSpline::ComputeSpline()
{
    const std::vector<Vector3d>& p = GetPoints();

    int N = (int)p.size()-1;
    _p1Points.resize(N);
    _p2Points.resize(N);
    if(N == 0)
        return false;

    if(N == 1)
    {  // Only 2 points...just create a straight line.
        // Constraint:  3*P1 = 2*P0 + P3
        _p1Points[0] = (2.0/3.0*p[0] + 1.0/3.0*p[1]);
        // Constraint:  P2 = 2*P1 - P0
        _p2Points[0] = 2.0*_p1Points[0] - p[0];
        return true;
    }

    /*rhs vector*/
    std::vector<Vector3d> a(N);
    std::vector<Vector3d> b(N);
    std::vector<Vector3d> c(N);
    std::vector<Vector3d> r(N);

    /*left most segment*/
    a[0].x() = 0;
    b[0].x() = 2;
    c[0].x() = 1;
    r[0].x() = p[0].x()+2*p[1].x();

    a[0].y() = 0;
    b[0].y() = 2;
    c[0].y() = 1;
    r[0].y() = p[0].y()+2*p[1].y();

    /*internal segments*/
    for (int i = 1; i < N - 1; i++)
    {
        a[i].x()=1;
        b[i].x()=4;
        c[i].x()=1;
        r[i].x() = 4 * p[i].x() + 2 * p[i+1].x();

        a[i].y()=1;
        b[i].y()=4;
        c[i].y()=1;
        r[i].y() = 4 * p[i].y() + 2 * p[i+1].y();
    }

    /*right segment*/
    a[N-1].x() = 2;
    b[N-1].x() = 7;
    c[N-1].x() = 0;
    r[N-1].x() = 8*p[N-1].x()+p[N].x();

    a[N-1].y() = 2;
    b[N-1].y() = 7;
    c[N-1].y() = 0;
    r[N-1].y() = 8*p[N-1].y()+p[N].y();


    /*solves Ax=b with the Thomas algorithm (from Wikipedia)*/
    for (int i = 1; i < N; i++)
    {
        double m;

        m = a[i].x()/b[i-1].x();
        b[i].x() = b[i].x() - m * c[i - 1].x();
        r[i].x() = r[i].x() - m * r[i-1].x();

        m = a[i].y()/b[i-1].y();
        b[i].y() = b[i].y() - m * c[i - 1].y();
        r[i].y() = r[i].y() - m * r[i-1].y();
    }

    _p1Points[N-1].x() = r[N-1].x()/b[N-1].x();
    _p1Points[N-1].y() = r[N-1].y()/b[N-1].y();
    for (int i = N - 2; i >= 0; --i)
    {
        _p1Points[i].x() = (r[i].x() - c[i].x() * _p1Points[i+1].x()) / b[i].x();
        _p1Points[i].y() = (r[i].y() - c[i].y() * _p1Points[i+1].y()) / b[i].y();
    }

    /*we have p1, now compute p2*/
    for (int i=0;i<N-1;i++)
    {
        _p2Points[i].x()=2*p[i+1].x()-_p1Points[i+1].x();
        _p2Points[i].y()=2*p[i+1].y()-_p1Points[i+1].y();
    }

    _p2Points[N-1].x() = 0.5 * (p[N].x()+_p1Points[N-1].x());
    _p2Points[N-1].y() = 0.5 * (p[N].y()+_p1Points[N-1].y());

    return true;
}

void BezierSpline::PrintDerivedData()
{
    std::cout << " Control Points " << std::endl;
    for(int idx = 0; idx < _p1Points.size(); idx++)
    {
        std::cout << "[" << idx << "]  ";
        std::cout << "P1: " << _p1Points[idx];
        std::cout << "   ";
        std::cout << "P2: " << _p2Points[idx];
        std::cout << std::endl;
    }
}

std::vector<Vector3d> BezierSpline::BuildSpline(std::vector<Vector3d> path, int divisions)
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