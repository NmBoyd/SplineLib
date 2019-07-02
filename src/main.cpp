#include "../lib/matplotlibcpp.h"
#include "../include/cubic_spline.h"
#include "../include/bezier_spline.h"
#include <vector>
#include <chrono>

using namespace Eigen;
int main(int argc, char** argv)
{
    std::vector<Vector3d> path; // input
    int divisions = 10; // input
    path.push_back(Vector3d(0.0,0.0,0.0));
    path.push_back(Vector3d(0.7,0.45,0.0));
    path.push_back(Vector3d(1.2,1.0,0.0));
    path.push_back(Vector3d(1.9,1.5,0.0));
    path.push_back(Vector3d(1.1,1.4,0.0));
    path.push_back(Vector3d(1.3,1.6,0.0));
    path.push_back(Vector3d(2.0,2.2,0.0));
    path.push_back(Vector3d(2.5,3.0,0.0));
    path.push_back(Vector3d(40.0,0.0,0.0));
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
    auto start = std::chrono::system_clock::now();
        b_spline.BuildSpline(path,divisions);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "Bezier spline finished computation in " 
              << elapsed_seconds.count() << "s\n" << std::endl;

    start = std::chrono::system_clock::now();
        c_spline.BuildSpline(path,divisions);
    end = std::chrono::system_clock::now();
    elapsed_seconds = end-start;
    end_time = std::chrono::system_clock::to_time_t(end);
 
    std::cout << "Cubic spline finished computation in " 
              << elapsed_seconds.count() << "s\n"<< std::endl;

    std::vector<double> b_pathx(b_spline.GetPositionProfile().size());
    std::vector<double> b_pathy(b_spline.GetPositionProfile().size());
    std::vector<double> b_pathvx(b_spline.GetVelocityProfile().size());
    std::vector<double> b_pathvy(b_spline.GetVelocityProfile().size());
    std::vector<double> b_pathax(b_spline.GetAccelerationProfile().size());
    std::vector<double> b_pathay(b_spline.GetAccelerationProfile().size());
    std::vector<double> b_pathc(b_spline.GetCurvatureProfile().size());

    std::vector<double> c_pathx(c_spline.GetPositionProfile().size());
    std::vector<double> c_pathy(c_spline.GetPositionProfile().size());
    std::vector<double> c_pathvx(c_spline.GetVelocityProfile().size());
    std::vector<double> c_pathvy(c_spline.GetVelocityProfile().size());
    std::vector<double> c_pathax(c_spline.GetAccelerationProfile().size());
    std::vector<double> c_pathay(c_spline.GetAccelerationProfile().size());
    std::vector<double> c_pathc(c_spline.GetCurvatureProfile().size());

    std::vector<double> t(b_spline.GetPositionProfile().size());
    std::vector<double> ti(c_spline.GetPositionProfile().size());
    for(int i=0;i<b_pathx.size();i++)
    {
        b_pathx[i] = b_spline.GetPositionProfile()[i].x();
        b_pathy[i] = b_spline.GetPositionProfile()[i].y();
        b_pathvx[i] = b_spline.GetVelocityProfile()[i].x();
        b_pathvy[i] = b_spline.GetVelocityProfile()[i].y();
        b_pathax[i] = b_spline.GetAccelerationProfile()[i].x();
        b_pathay[i] = b_spline.GetAccelerationProfile()[i].y();
        b_pathc[i] = b_spline.GetCurvatureProfile()[i];
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
        c_pathc[i] = c_spline.GetCurvatureProfile()[i];
        ti[i] = i/(double)divisions;
    }
    std::cout << c_spline.EvaluateCurveLength() << std::endl;
    c_spline.PrintData(5);
    b_spline.PrintData(5);
    matplotlibcpp::figure(1);
    matplotlibcpp::title("X-Y Interpolation");
    matplotlibcpp::ylabel("Y-Position [m]");
    matplotlibcpp::xlabel("X-Position [m]");
    matplotlibcpp::plot(x_orig, y_orig, "x-");
    matplotlibcpp::plot(b_pathx, b_pathy);  
    matplotlibcpp::plot(c_pathx, c_pathy, "r-");   // show plots
    matplotlibcpp::figure(2);
    matplotlibcpp::title("Tangential Velocity");
    matplotlibcpp::ylabel("Velocity [m/s]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, b_pathvx);  
    matplotlibcpp::plot(ti, c_pathvx, "r-");   
    matplotlibcpp::figure(3);
    matplotlibcpp::title("Tangential Acceleration");
    matplotlibcpp::ylabel("Acceleration [m/s^2]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, b_pathax);   
    matplotlibcpp::plot(ti, c_pathax, "r-");  
    matplotlibcpp::figure(4);
    matplotlibcpp::title("Spline Curvature");
    matplotlibcpp::ylabel("Curvature [1/m]");
    matplotlibcpp::xlabel("time [s]");
    matplotlibcpp::plot(t, b_pathc);   
    matplotlibcpp::plot(ti, c_pathc, "r-"); 
    matplotlibcpp::show();

}
