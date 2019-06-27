#ifndef __SplineCommon__
#define __SplineCommon__

#include "CommonSTL.h"
#include "CommonProject.h"
#include "MathUtilities.h"

/* A Spline base class. */
class SplineBase
{
private:
   vector<Vec2> _points;
   bool _elimColinearPoints;

protected:


protected:
   /* OVERRIDE THESE FUNCTIONS */
   virtual void ResetDerived() = 0;

   enum
   {
      NOM_SIZE = 32,
   };

public:

   SplineBase()
   {
      _points.reserve(NOM_SIZE);
      _elimColinearPoints = true;
   }

   const vector<Vec2>& GetPoints() { return _points; }
   bool GetElimColinearPoints() { return _elimColinearPoints; }
   void SetElimColinearPoints(bool elim) { _elimColinearPoints = elim; }


   /* OVERRIDE THESE FUNCTIONS */
   virtual Vec2 Eval(int seg, double t) = 0;
   virtual bool ComputeSpline() = 0;
   virtual void DumpDerived() {}

   /* Clear out all the data.
    */
   void Reset()
   {
      _points.clear();
      ResetDerived();
   }

   void AddPoint(const Vec2& pt)
   {
      // If this new point is colinear with the two previous points,
      // pop off the last point and add this one instead.
      if(_elimColinearPoints && _points.size() > 2)
      {
         int N = _points.size()-1;
         Vec2 p0 = _points[N-1] - _points[N-2];
         Vec2 p1 = _points[N] - _points[N-1];
         Vec2 p2 = pt - _points[N];
         // We test for colinearity by comparing the slopes
         // of the two lines.  If the slopes are the same,
         // we assume colinearity.
         float32 delta = (p2.y-p1.y)*(p1.x-p0.x)-(p1.y-p0.y)*(p2.x-p1.x);
         if(MathUtilities::IsNearZero(delta))
         {
            _points.pop_back();
         }
      }
      _points.push_back(pt);
   }

   void Dump(int segments = 5)
   {
      assert(segments > 1);

      cout << "Original Points (" << _points.size() << ")" << endl;
      cout << "-----------------------------" << endl;
      for(int idx = 0; idx < _points.size(); ++idx)
      {
         cout << "[" << idx << "]" << "  " << _points[idx] << endl;
      }

      cout << "-----------------------------" << endl;
      DumpDerived();

      cout << "-----------------------------" << endl;
      cout << "Evaluating Spline at " << segments << " points." << endl;
      for(int idx = 0; idx < _points.size()-1; idx++)
      {
         cout << "---------- " << "From " <<  _points[idx] << " to " << _points[idx+1] << "." << endl;
         for(int tIdx = 0; tIdx < segments+1; ++tIdx)
         {
            double t = tIdx*1.0/segments;
            cout << "[" << tIdx << "]" << "   ";
            cout << "[" << t*100 << "%]" << "   ";
            cout << " --> " << Eval(idx,t);
            cout << endl;
         }
      }
   }
};

class ClassicSpline : public SplineBase
{
private:
   /* The system of linear equations found by solving
    * for the 3 order spline polynomial is given by:
    * A*x = b.  The "x" is represented by _xCol and the
    * "b" is represented by _bCol in the code.
    *
    * The "A" is formulated with diagonal elements (_diagElems) and
    * symmetric off-diagonal elements (_offDiagElemns).  The
    * general structure (for six points) looks like:
    *
    *
    *  |  d1  u1   0   0   0  |      | p1 |    | w1 |
    *  |  u1  d2   u2  0   0  |      | p2 |    | w2 |
    *  |  0   u2   d3  u3  0  |   *  | p3 |  = | w3 |
    *  |  0   0    u3  d4  u4 |      | p4 |    | w4 |
    *  |  0   0    0   u4  d5 |      | p5 |    | w5 |
    *
    *
    *  The general derivation for this can be found
    *  in Robert Sedgewick's "Algorithms in C++".
    *
    */
   vector<double> _xCol;
   vector<double> _bCol;
   vector<double> _diagElems;
   vector<double> _offDiagElems;
public:
   ClassicSpline()
   {
      _xCol.reserve(NOM_SIZE);
      _bCol.reserve(NOM_SIZE);
      _diagElems.reserve(NOM_SIZE);
      _offDiagElems.reserve(NOM_SIZE);
   }

   /* Evaluate the spline for the ith segment
    * for parameter.  The value of parameter t must
    * be between 0 and 1.
    */
   inline virtual Vec2 Eval(int seg, double t)
   {
      const vector<Vec2>& points = GetPoints();

      assert(t >= 0);
      assert(t <= 1.0);
      assert(seg >= 0);
      assert(seg < (points.size()-1));

      const double ONE_OVER_SIX = 1.0/6.0;
      double oneMinust = 1.0 - t;
      double t3Minust = t*t*t-t;
      double oneMinust3minust = oneMinust*oneMinust*oneMinust-oneMinust;
      double deltaX = points[seg+1].x - points[seg].x;
      double yValue = t * points[seg + 1].y +
      oneMinust*points[seg].y +
      ONE_OVER_SIX*deltaX*deltaX*(t3Minust*_xCol[seg+1] - oneMinust3minust*_xCol[seg]);
      double xValue = t*(points[seg+1].x-points[seg].x) + points[seg].x;
      return Vec2(xValue,yValue);
   }


   /* Clear out all the data.
    */
   virtual void ResetDerived()
   {
      _diagElems.clear();
      _bCol.clear();
      _xCol.clear();
      _offDiagElems.clear();
   }


   virtual bool ComputeSpline()
   {
      const vector<Vec2>& p = GetPoints();


      _bCol.resize(p.size());
      _xCol.resize(p.size());
      _diagElems.resize(p.size());

      for(int idx = 1; idx < p.size(); ++idx)
      {
         _diagElems[idx] = 2*(p[idx+1].x-p[idx-1].x);
      }
      for(int idx = 0; idx < p.size(); ++idx)
      {
         _offDiagElems[idx] = p[idx+1].x - p[idx].x;
      }
      for(int idx = 1; idx < p.size(); ++idx)
      {
         _bCol[idx] = 6.0*((p[idx+1].y-p[idx].y)/_offDiagElems[idx] -
                           (p[idx].y-p[idx-1].y)/_offDiagElems[idx-1]);
      }
      _xCol[0] = 0.0;
      _xCol[p.size()-1] = 0.0;
      for(int idx = 1; idx < p.size()-1; ++idx)
      {
         _bCol[idx+1] = _bCol[idx+1] - _bCol[idx]*_offDiagElems[idx]/_diagElems[idx];
         _diagElems[idx+1] = _diagElems[idx+1] - _offDiagElems[idx]*_offDiagElems[idx]/_diagElems[idx];
      }
      for(int idx = (int)p.size()-2; idx > 0; --idx)
      {
         _xCol[idx] = (_bCol[idx] - _offDiagElems[idx]*_xCol[idx+1])/_diagElems[idx];
      }
      return true;
   }
};

/* Bezier Spline Implementation
 * Based on this article:
 * http://www.particleincell.com/blog/2012/bezier-splines/
 */
class BezierSpine : public SplineBase
{
private:
   vector<Vec2> _p1Points;
   vector<Vec2> _p2Points;
public:
   BezierSpine()
   {
      _p1Points.reserve(NOM_SIZE);
      _p2Points.reserve(NOM_SIZE);
   }

   /* Evaluate the spline for the ith segment
    * for parameter.  The value of parameter t must
    * be between 0 and 1.
    */
   inline virtual Vec2 Eval(int seg, double t)
   {
      assert(seg < _p1Points.size());
      assert(seg < _p2Points.size());

      double omt = 1.0 - t;

      Vec2 p0 = GetPoints()[seg];
      Vec2 p1 = _p1Points[seg];
      Vec2 p2 = _p2Points[seg];
      Vec2 p3 = GetPoints()[seg+1];

      double xVal = omt*omt*omt*p0.x + 3*omt*omt*t*p1.x +3*omt*t*t*p2.x+t*t*t*p3.x;
      double yVal = omt*omt*omt*p0.y + 3*omt*omt*t*p1.y +3*omt*t*t*p2.y+t*t*t*p3.y;
      return Vec2(xVal,yVal);
   }

   /* Clear out all the data.
    */
   virtual void ResetDerived()
   {
      _p1Points.clear();
    
        path2[i].x() = path[i].x();
        path2[i].y() = path[i].y();
   }
        path2[i].x() = path[i].x();
        path2[i].y() = path[i].y();


   v
        path2[i].x() = path[i].x();
        path2[i].y() = path[i].y();e()
   {
        path2[i].x() = path[i].x();
        path2[i].y() = path[i].y();
      const vector<Vec2>& p = GetPoints();

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
      vector<Vec2> a(N);
      vector<Vec2> b(N);
      vector<Vec2> c(N);
      vector<Vec2> r(N);

      /*left most segment*/
      a[0].x = 0;
      b[0].x = 2;
      c[0].x = 1;
      r[0].x = p[0].x+2*p[1].x;

      a[0].y = 0;
      b[0].y = 2;
      c[0].y = 1;
      r[0].y = p[0].y+2*p[1].y;

      /*internal segments*/
      for (int i = 1; i < N - 1; i++)
      {
         a[i].x=1;
         b[i].x=4;
         c[i].x=1;
         r[i].x = 4 * p[i].x + 2 * p[i+1].x;

         a[i].y=1;
         b[i].y=4;
         c[i].y=1;
         r[i].y = 4 * p[i].y + 2 * p[i+1].y;
      }

      /*right segment*/
      a[N-1].x = 2;
      b[N-1].x = 7;
      c[N-1].x = 0;
      r[N-1].x = 8*p[N-1].x+p[N].x;

      a[N-1].y = 2;
      b[N-1].y = 7;
      c[N-1].y = 0;
      r[N-1].y = 8*p[N-1].y+p[N].y;


      /*solves Ax=b with the Thomas algorithm (from Wikipedia)*/
      for (int i = 1; i < N; i++)
      {
         double m;

         m = a[i].x/b[i-1].x;
         b[i].x = b[i].x - m * c[i - 1].x;
         r[i].x = r[i].x - m * r[i-1].x;

         m = a[i].y/b[i-1].y;
         b[i].y = b[i].y - m * c[i - 1].y;
         r[i].y = r[i].y - m * r[i-1].y;
      }

      _p1Points[N-1].x = r[N-1].x/b[N-1].x;
      _p1Points[N-1].y = r[N-1].y/b[N-1].y;
      for (int i = N - 2; i >= 0; --i)
      {
         _p1Points[i].x = (r[i].x - c[i].x * _p1Points[i+1].x) / b[i].x;
         _p1Points[i].y = (r[i].y - c[i].y * _p1Points[i+1].y) / b[i].y;
      }

      /*we have p1, now compute p2*/
      for (int i=0;i<N-1;i++)
      {
         _p2Points[i].x=2*p[i+1].x-_p1Points[i+1].x;
         _p2Points[i].y=2*p[i+1].y-_p1Points[i+1].y;
      }

      _p2Points[N-1].x = 0.5 * (p[N].x+_p1Points[N-1].x);
      _p2Points[N-1].y = 0.5 * (p[N].y+_p1Points[N-1].y);

      return true;
   }

   virtual void DumpDerived()
   {
      cout << " Control Points " << endl;
      for(int idx = 0; idx < _p1Points.size(); idx++)
      {
         cout << "[" << idx << "]  ";
         cout << "P1: " << _p1Points[idx];
         cout << "   ";
         cout << "P2: " << _p2Points[idx];
         cout << endl;
      }
   }
};




//////////////////////////////////
// Example

/* Smooth the points on the path so that turns look
 * more natural.  We'll only smooth the first few 
 * points.  Most of the time, the full path will not
 * be executed anyway...why waste cycles.
 */
void SmoothPath(vector<Vec2>& path, int32 divisions)
{
   const int SMOOTH_POINTS = 6;

   BezierSpine spline;

   if(path.size() < 2)
      return;

   // Cache off the first point.  If the first point is removed,
   // the we occasionally run into problems if the collision detection
   // says the first node is occupied but the splined point is too
   // close, so the FSM "spins" trying to find a sensor cell that is
   // not occupied.
   //   Vec2 firstPoint = path.back();
   //   path.pop_back();
   // Grab the points.
   for(int idx = 0; idx < SMOOTH_POINTS && path.size() > 0; idx++)
   {
      spline.AddPoint(path.back());
      path.pop_back();
   }
   // Smooth them.
   spline.ComputeSpline();
   // Push them back in.
   for(int idx = spline.GetPoints().size()-2; idx >= 0; --idx)
   {
      for(int division = divisions-1; division >= 0; --division)
      {
         double t = division*1.0/divisions;
         path.push_back(spline.Eval(idx, t));
      }
   }
   // Push back in the original first point.
   //   path.push_back(firstPoint);
}