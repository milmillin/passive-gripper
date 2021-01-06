#include "Geometry.h"

#include <cmath>
#include <iostream>

namespace gripper {

double HorizontalTwiceSignedArea(const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  return (t1(0) - t3(0)) * (t2(1) - t3(1)) - (t2(0) - t3(0)) * (t1(1) - t3(1));
}

bool IsSupportPointStable(const Vector3d &p,
    const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  double d1 = HorizontalTwiceSignedArea(p, t1, t2);
  double d2 = HorizontalTwiceSignedArea(p, t2, t3);
  double d3 = HorizontalTwiceSignedArea(p, t3, t1);
  std::cout << "IsSupportPointStable " << d1 << " " << d2 << " " << d3 << std::endl;
  return (d1 > 0 && d2 > 0 && d3 > 0) || (d1 < 0 && d2 < 0 && d3 < 0);
}

double HorizontalDistance(const Vector3d &p, const Vector3d &l1, const Vector3d &l2) {
  Vector2d v(  l2(1) - l1(1),
             -(l2(0) - l1(0)));
  Vector2d r(l1(0) - p(0), l1(1) - p(1));
  std::cout <<"vector "<< v(0) <<" " <<v(1)<<" " <<r(0)<<" "<<r(1)<<std::endl;
  std::cout <<"dot " << v.dot(r);
  std::cout <<"abs " << std::abs(v.dot(r));
  return std::abs(v.dot(r));
}

double TriangleStability(const Vector3d &p,
    const Vector3d &t1, const Vector3d &t2, const Vector3d &t3) {
  if (!IsSupportPointStable(p, t1, t2, t3)) {
    return 0;
  }
  double d1 = HorizontalDistance(p, t1, t2);
  double d2 = HorizontalDistance(p, t2, t3);
  double d3 = HorizontalDistance(p, t3, t1);
  std::cout << "TriangleStability " << d1 << " " << d2 << " " << d3 << std::endl;
  return std::min({d1, d2, d3});
}

}  // namespace gripper