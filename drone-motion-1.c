#include <stdio.h>
#include <math.h>

double ff(double);
double fu(double);
double fv(double);
double fx(double);
double fy(double);

double m = 1;
double g = 3.711;
double theta = 4 * M_PI / 9;

int main()
{
  double t_i = 0e0;
  double t_f = 5e0;
  double dt = 1e-1;
  double t;

  double u_i = 0;
  double v_i = 0;
  double x_i = 0;
  double y_i = 0;

  double u;
  double v;
  double x;
  double y;
  
  double uk1, uk2, uk3, uk4;
  double vk1, vk2, vk3, vk4;
  double xk1, xk2, xk3, xk4;
  double yk1, yk2, yk3, yk4;

  FILE *fp;
  fp = fopen("data.csv", "w");
  for(t = t_i; t < t_f; t += dt)
  {
    fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", u, v, x, y);
    uk1 = dt * fu(t);
    uk2 = dt * fu(t + dt / 2);
    uk3 = dt * fu(t + dt / 2);
    uk4 = dt * fu(t + dt);

    vk1 = dt * fv(t);
    vk2 = dt * fv(t + dt / 2);
    vk3 = dt * fv(t + dt / 2);
    vk4 = dt * fv(t + dt);

    xk1 = dt * fx(u);
    xk2 = dt * fx(u + xk1 / 2);
    xk3 = dt * fx(u + xk2 / 2);
    xk4 = dt * fx(u + xk3);

    yk1 = dt * fy(v);
    yk2 = dt * fy(v + yk1 / 2);
    yk3 = dt * fy(v + yk2 / 2);
    yk4 = dt * fy(v + yk3);

    u += (uk1 + 2 * uk2 + 2 * uk3 + uk4);
    v += (vk1 + 2 * vk2 + 2 * vk3 + vk4);
    x += (xk1 + 2 * xk2 + 2 * xk3 + xk4);
    y += (yk1 + 2 * yk2 + 2 * yk3 + yk4);
  }
  fprintf(fp, "%lf\t%lf\t%lf\t%lf\n", u, v, x, y);
  fclose(fp);

  return 0;
}

double ff(double t)
{
  if(t < 1) return 10;
  else return 0;
}

double fu(double t)
{
  double u_num;
  u_num = ff(t) * cos(theta) / m;
  return u_num;
}

double fv(double t)
{
  double v_num;
  v_num = ff(t) * sin(theta) / m - g;
  return v_num;
}

double fx(double u)
{
  return u;
}

double fy(double v)
{
  return v;
}
