// #include "../lib/matplotlibcpp.h"
#include "../include/cubic_spline.h"
#include "../include/bezier_spline.h"

using namespace Eigen;
int main(int argc, char** argv)
{
    std::vector<Vector3d> path; // input
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
    
    std::vector<Vector3d> b_path(b_spline.BuildSpline(path,50));
    std::vector<Vector3d> c_path(c_spline.BuildSpline(path,50));
    std::vector<double> b_pathx(b_path.size());
    std::vector<double> b_pathy(b_path.size());
    std::vector<double> c_pathx(c_path.size());
    std::vector<double> c_pathy(c_path.size());
    for(int i=0;i<b_pathx.size();++i)
    {
        b_pathx[i] = b_path[i].x();
        b_pathy[i] = b_path[i].y();
    }
    for(int i=0;i<c_pathx.size();++i)
    {
        c_pathx[i] = c_path[i].x();
        c_pathy[i] = c_path[i].y();
    }
    c_spline.PrintData(5); 
    // matplotlibcpp::plot(x_orig, y_orig,"x");
    // matplotlibcpp::plot(c_pathx, c_pathy,"r-");   // show plots
    // matplotlibcpp::plot(b_pathx, b_pathy);   // show plots
    // matplotlibcpp::show();

}
