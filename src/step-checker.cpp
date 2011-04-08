#include <cmath>

#include <sot-pattern-generator/step-checker.h>

namespace dynamicgraph {
  namespace sot {

    void StepChecker::clipStep(double x, double y, double & x_result, double & y_result)
    {
      //   const double MIN_y = 0.16;
      //   const double MAX_y = 0.40;
      //   const double MAX_x = 0.25;
      //   const double EDGE_POINT_x = 0.20;
      //   const double EDGE_POINT_y = 0.30;

      const double MIN_y = 0.16;
      const double MAX_y = 0.30;
      const double MAX_x = 0.25;
      const double EDGE_POINT_x = 0.20;
      const double EDGE_POINT_y = 0.30;

      double x0 = std::abs(x);
      double y0 = std::abs(y);
      double alpha = y0/x0;
      double y_inter = MIN_y;
      double x_inter = MIN_y/alpha;

      if(alpha < MIN_y/MAX_x) {
	x_result = MAX_x;
	y_result = MIN_y;
      }
      else if(alpha < EDGE_POINT_y / EDGE_POINT_x) {
	double delta1 = (MIN_y - EDGE_POINT_y) / (MAX_x - EDGE_POINT_x);
	double x_inter2 = (EDGE_POINT_y - delta1 * EDGE_POINT_x) / (alpha - delta1);
	double y_inter2 = alpha * x_inter2;
	if(x0 < x_inter) {
	  x_result = x_inter;
	  y_result = y_inter;
	}
	else if(x0 > x_inter2) {
	  x_result = x_inter2;
	  y_result = y_inter2;
	}
	else {
	  x_result = x0;
	  y_result = y0;
	}
      }
      else {
	double delta2 = (EDGE_POINT_y - MAX_y) / (EDGE_POINT_x);
	double x_inter2 = (MAX_y) / (alpha - delta2);
	double y_inter2 = alpha * x_inter2;
	if(x0 < x_inter) {
	  x_result = x_inter;
	  y_result = y_inter;
	}
	else if(x0 > x_inter2) {
	  x_result = x_inter2;
	  y_result = y_inter2;
	}
	else {
	  x_result = x0;
	  y_result = y0;
	}
      }

      if(x<0) x_result = -x_result;
      if(y<0) y_result = -y_result;
    }

  } // namespace dg
} // namespace sot
