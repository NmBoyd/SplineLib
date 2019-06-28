// #include "../lib/matplotlibcpp.h"
#include "../include/cubic_spline.h"
#include "../include/bezier_spline.h"

using namespace Eigen;
int main(int argc, char** argv)
{
    std::vector<Vector3d> path; // input
<<<<<<< HEAD:src/main.cpp
    int divisions = 10; // input
    path.push_back(Vector3d(0.0,0.0,0.0));
    path.push_back(Vector3d(0.7,0.45,0.0));
    path.push_back(Vector3d(1.2,1.0,0.0));
    path.push_back(Vector3d(1.9,1.5,0.0));
    path.push_back(Vector3d(1.1,1.4,0.0));
    path.push_back(Vector3d(1.3,1.6,0.0));
    path.push_back(Vector3d(2.0,2.2,0.0));
    path.push_back(Vector3d(2.5,3.0,0.0));
    path.push_back(Vector3d(4.0,0.0,0.0));
=======
    int divisions = 50; // input
    // path.push_back(Vector3d(0.0,0.0,0.0));
    // path.push_back(Vector3d(0.7,0.45,0.0));
    // path.push_back(Vector3d(1.2,1.0,0.0));
    // path.push_back(Vector3d(1.9,1.5,0.0));
    // path.push_back(Vector3d(1.1,1.4,0.0));
    // path.push_back(Vector3d(1.3,1.6,0.0));
    // path.push_back(Vector3d(2.0,2.2,0.0));
    // path.push_back(Vector3d(2.5,3.0,0.0));
    // path.push_back(Vector3d(4.0,0.0,0.0));
    path.push_back(Vector3d(0.0,0.0,0.0));
    path.push_back(Vector3d(0.2,0.2,0.0));
    path.push_back(Vector3d(0.4,0.3,0.0));
    path.push_back(Vector3d(0.6,0.35,0.0));
    path.push_back(Vector3d(0.8,0.3,0.0));
    path.push_back(Vector3d(1.0,0.2,0.0));
    path.push_back(Vector3d(1.2,0.0,0.0));
>>>>>>> master:examples/example.cpp
    std::vector<double> x_orig(path.size());
    std::vector<double> y_orig(path.size());
    for(int i=0;i<path.size();i++)
    {
        x_orig[i] = path[i].x();
        y_orig[i] = path[i].y();
    }
    BezierSpline b_spline;
    CubicSpline c_spline;

    if(path.size() < 2)
        return 0;
    b_spline.BuildSpline(path,10);
    c_spline.BuildSpline(path,10);

    std::vector<double> b_pathx(b_spline.GetPositionProfile().size());
    std::vector<double> b_pathy(b_spline.GetPositionProfile().size());
    std::vector<double> b_pathvx(b_spline.GetVelocityProfile().size());
    std::vector<double> b_pathvy(b_spline.GetVelocityProfile().size());
    std::vector<double> b_pathax(b_spline.GetAccelerationProfile().size());
    std::vector<double> b_pathay(b_spline.GetAccelerationProfile().size());

    std::vector<double> c_pathx(c_spline.GetPositionProfile().size());
    std::vector<double> c_pathy(c_spline.GetPositionProfile().size());
    std::vector<double> c_pathvx(c_spline.GetVelocityProfile().size());
    std::vector<double> c_pathvy(c_spline.GetVelocityProfile().size());
    std::vector<double> c_pathax(c_spline.GetAccelerationProfile().size());
    std::vector<double> c_pathay(c_spline.GetAccelerationProfile().size());

    std::vector<double> t(b_spline.GetPositionProfile().size());
    for(int i=0;i<b_pathx.size();i++)
    {
        b_pathx[i] = b_spline.GetPositionProfile()[i].x();
        b_pathy[i] = b_spline.GetPositionProfile()[i].y();
        b_pathvx[i] = b_spline.GetVelocityProfile()[i].x();
        b_pathvy[i] = b_spline.GetVelocityProfile()[i].y();
        b_pathax[i] = b_spline.GetAccelerationProfile()[i].x();
        b_pathay[i] = b_spline.GetAccelerationProfile()[i].y();
        t[i] = i/(double)divisions;
    }
    for(int i=0;i<c_pathx.size();++i)
    {
        c_pathx[i] = c_spline.GetPositionProfile()[i].x();
        c_pathy[i] = c_spline.GetPositionProfile()[i].y();
        c_pathvx[i] = c_spline.GetVelocityProfile()[i].x();
        c_pathvy[i] = c_spline.GetVelocityProfile()[i].y();
        c_pathax[i] = c_spline.GetAccelerationProfile()[i].x();
        c_pathay[i] = c_spline.GetAccelerationProfile()[i].y();
    }
<<<<<<< HEAD:src/main.cpp
    matplotlibcpp::figure(1);
    matplotlibcpp::plot(x_orig, y_orig,"x");
    matplotlibcpp::plot(c_pathx, c_pathy,"r-");   // show plots
    matplotlibcpp::plot(b_pathx, b_pathy);   // show plots
    matplotlibcpp::figure(2);
    matplotlibcpp::plot(t, b_pathvx);   // show plots
    matplotlibcpp::plot(t, c_pathvx, "r-");   // show plots
    matplotlibcpp::figure(3);
    matplotlibcpp::plot(t, b_pathax);   // show plots
    matplotlibcpp::plot(t, c_pathvx, "r-");   // show plots
    matplotlibcpp::show();
=======
    c_spline.PrintData(5); 
    // matplotlibcpp::plot(x_orig, y_orig,"x");
    // matplotlibcpp::plot(c_pathx, c_pathy,"r-");   // show plots
    // matplotlibcpp::plot(b_pathx, b_pathy);   // show plots
    // matplotlibcpp::show();
>>>>>>> master:examples/example.cpp

}
