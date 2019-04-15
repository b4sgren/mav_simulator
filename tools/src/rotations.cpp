#include <tools/rotations.h>
#include <cmath>

namespace tools
{
  Eigen::Vector3d Quaternion2Euler(const Eigen::Vector4d& e)
  {
    double e0{e(0)}, e1{e(1)}, e2{e(2)}, e3{e(3)};

    Eigen::Vector3d euler;
    euler(0) = atan2(2 * (e0 * e1 + e2 * e3), e0*e0 + e3*e3 - e1*e1 - e2*e2);
    euler(1) = asin(2 * (e0*e2 - e1*e3));
    euler(2) = atan2(2 * (e0*e3 + e1*e2), e0*e0 + e1*e1 - e2*e2 - e3*e3);

    return euler;
  }

  Eigen::Vector4d Euler2Quaternion(const Eigen::Vector3d& euler)
  {
    double phi{euler(0)}, theta{euler(1)}, psi{euler(2)};
    double cphi{cos(phi/2.0)}, sphi{sin(phi/2.0)};
    double ctheta{cos(theta/2.0)}, stheta{sin(theta/2.0)};
    double cpsi{cos(psi/2.0)}, spsi{sin(theta/2.0)};

    Eigen::Vector4d e;
    e(0) = cpsi * ctheta*cphi + spsi*stheta*sphi;
    e(1) = cpsi*ctheta*sphi - spsi*stheta*cphi;
    e(2) = cpsi*stheta*cphi + spsi*ctheta*sphi;
    e(3) = spsi*ctheta*cphi - cpsi*stheta*sphi;

    return e;
  }

  Eigen::Matrix3d Quaternion2Rotation(const Eigen::Vector4d& e)
  {
    //Returns R_b2i
    double e0{e(0)}, e1{e(1)}, e2{e(2)}, e3{e(3)};

    double var1 = e0*e0 + e1*e1 - e2*e2 - e3*e3;
    double var2 = 2 * (e1*e2 - e0*e3);
    double var3 = 2 * (e1*e3 + e0*e2);
    double var4 = 2 * (e1*e2 + e0*e3);
    double var5 = e0*e0 - e1*e1 + e2*e2 - e3*e3;
    double var6 = 2 * (e2*e3 - e0*e1);
    double var7 = 2 * (e1*e3 - e0*e2);
    double var8 = 2 * (e2*e3 + e0*e1);
    double var9 = e0*e0 - e1*e1 - e2*e2 + e3*e3;

    Eigen::Matrix3d R;
    R << var1, var2, var3, var4, var5, var6, var7, var8, var9;

    return R;
  }

  Eigen::Matrix3d Euler2Rotation(const Eigen::Vector3d& euler)
  {
    Eigen::Vector4d e = Euler2Quaternion(euler);
    Eigen::Matrix3d R = Quaternion2Rotation(e);

    return R;
  }

  Eigen::Matrix4d skew(const Eigen::Vector3d& w)
  {
    double p{w(0)}, q{w(1)}, r{w(2)};
    Eigen::Matrix4d temp;
    temp << 0, -p, -q, -r, p, 0, r, -q, q, -r, 0, p, r, q, -p, 0;

    return temp;
  }

  int sign(double a)
  {
    if(a >= 0)
      return 1;
    else
      return -1;
  }

  double norm(const Eigen::Vector3d& v)
  {
    return sqrt(v.transpose() * v);
  }
}
