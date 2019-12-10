/****************/


#include <stdio.h>
#include <math.h>

double ff(double, double);
double fu(double, double, double, double);
double fv(double, double, double, double);
double fx(double, double);
double fy(double, double);

double m = 1;
double g = 3.711;
double theta = 4 * M_PI / 9;

int main()
{
  double t_i = 0e0;
  double t_f = 10e0;
  double dt = 1e-4;
  double t;
  double t0 = 0e0;

  double u_i = 0;
  double v_i = 0;
  double x_i = 0;
  double y_i = 0;

  double u;
  double v;
  double x;
  double y;
  double y_o = y_i;;
  
  double uk1, uk2, uk3, uk4;
  double vk1, vk2, vk3, vk4;
  double xk1, xk2, xk3, xk4;
  double yk1, yk2, yk3, yk4;

  FILE *fp;
  fp = fopen("data.txt", "w");
  for(t = t_i; t < t_f; t += dt)
  {
    if(y_o * y < 0)
    {
      t0 = t;
      u = 0;
      v = 0;
      y = 0;
    }
    y_o = y;
    fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", t, u, v, x, y);
    uk1 = dt * fu(t, x, y, t0);
    uk2 = dt * fu(t + dt / 2, x, y, t0);
    uk3 = dt * fu(t + dt / 2, x, y, t0);
    uk4 = dt * fu(t + dt, x, y, t0);

    vk1 = dt * fv(t, x, y, t0);
    vk2 = dt * fv(t + dt / 2, x, y, t0);
    vk3 = dt * fv(t + dt / 2, x, y, t0);
    vk4 = dt * fv(t + dt, x, y, t0);

    xk1 = dt * fx(y, u);
    xk2 = dt * fx(y, u + xk1 / 2);
    xk3 = dt * fx(y, u + xk2 / 2);
    xk4 = dt * fx(y, u + xk3);

    yk1 = dt * fy(y, v);
    yk2 = dt * fy(y, v + yk1 / 2);
    yk3 = dt * fy(y, v + yk2 / 2);
    yk4 = dt * fy(y, v + yk3);

    u += (uk1 + 2 * uk2 + 2 * uk3 + uk4);
    v += (vk1 + 2 * vk2 + 2 * vk3 + vk4);
    x += (xk1 + 2 * xk2 + 2 * xk3 + xk4);
    y += (yk1 + 2 * yk2 + 2 * yk3 + yk4);
  }
  fprintf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\n", t, u, v, x, y);
  fclose(fp);

  return 0;
}

double ff(double t, double t0)
{
  double phi = M_PI * fabs(t - t0) / 10;
  if(fabs(t - t0) < 1) return 5 * exp(t - t0) * sin(phi);
  else return 0;
}

double fu(double t, double x, double y, double t0)
{
  double u_num;
  u_num = ff(t, t0) * cos(theta) / m;
  if(y <= 0) return 0;
  else return u_num;
}

double fv(double t, double x, double y, double t0)
{
  double v_num;
  v_num = ff(t, t0) * sin(theta) / m - g;
  if(y <= 0 && v_num < 0) return 0;
  else return v_num;
}

double fx(double y, double u)
{
  if(y < 0) return 0;
  else return u;
}

double fy(double y, double v)
{
  if(y < 0) return 0;
  else return v;
}
