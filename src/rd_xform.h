#ifndef RD_XFORM_H
#define RD_XFORM_H

const int RD_XFORM_HSIZE = 4;
const int RD_XFORM_VSIZE = 4;

// Forward declarations for function declarations
class point;
class pointh;
class vector;

class rd_xform
{
public:
  double matrix[RD_XFORM_HSIZE][RD_XFORM_VSIZE];
  // Matrix Format:
  //   X 0 1 2 3
  // Y
  // 0   0 0 0 0
  // 1   0 0 0 0
  // 2   0 0 0 0
  // 3   0 0 0 0
  //
  // Where matrix[0][0] is the top left
  //       matrix[3][3] is the bottom right
  //       matrix[3][0] is the top right
  //       matrix[0][3] is the bottom left

  rd_xform();

  // Transformation Functions
  static rd_xform identity();
  static rd_xform identity(rd_xform m);
  static rd_xform multiply(const rd_xform& m1, const rd_xform& m2);
  static pointh   multiply(const rd_xform& m, const pointh& p);

  static rd_xform translate(const rd_xform& m, double tx, double ty, double tz);
  static rd_xform scale(const rd_xform& m, double sx, double sy, double sz);
  static rd_xform rotate_xy(const rd_xform& m, double theta);
  static rd_xform rotate_yz(const rd_xform& m, double theta);
  static rd_xform rotate_zx(const rd_xform& m, double theta);

  // Pipeline Functions
  static rd_xform world_to_camera(const point& eye, const point& at, const vector& up);
  static rd_xform camera_to_clip(double fov, double near, double far, double aspect);
  static rd_xform clip_to_device(int width, int height);
};//end class

#endif
