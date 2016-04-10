#include <cmath>
#include "point.h"
#include "pointh.h"
#include "rd_xform.h"
#include "vector.h"

const double PI = 3.14159265;

rd_xform::rd_xform()
{
  for(int i = 0; i < RD_XFORM_HSIZE; i++)
  {
    for(int j = 0; j < RD_XFORM_VSIZE; j++)
    {
      matrix[i][j] = 0;
    }//end for
  }//end for
}//end default constructor

rd_xform rd_xform::identity()
{
  rd_xform temp;
  for(int i = 0; i < RD_XFORM_HSIZE && i < RD_XFORM_VSIZE; i++)
  {
    temp.matrix[i][i] = 1;
  }//end for
  return temp;
}//end identity()

rd_xform rd_xform::identity(rd_xform m)
{
  for(int i = 0; i < RD_XFORM_HSIZE; i++)
  {
    for(int j = 0; j < RD_XFORM_VSIZE; j++)
    {
      if(i == j)
      {
	m.matrix[i][j] = 1;
      }//end if
      else
      {
	m.matrix[i][j] = 0;
      }//end else
    }//end for
  }//end for
  return m;
}//end identity(rd_xform)

rd_xform rd_xform::multiply(const rd_xform& m1, const rd_xform& m2)
{
  rd_xform temp;

  for(int i = 0; i < RD_XFORM_HSIZE; i++)
  {
    for(int j = 0; j < RD_XFORM_VSIZE; j++)
    {
      temp.matrix[i][j] = 0;
      for(int k = 0; k < RD_XFORM_HSIZE && k < RD_XFORM_VSIZE; k++)
      {
	temp.matrix[i][j] += m1.matrix[k][j] * m2.matrix[i][k];
      }//end for
    }//end for
  }//end for
  return temp;
}//end multiply(rd_xform, rd_xform)

pointh rd_xform::multiply(const rd_xform& m, const pointh& p)
{
  pointh temp;

  temp.x = (m.matrix[0][0] * p.x) +
           (m.matrix[1][0] * p.y) +
           (m.matrix[2][0] * p.z) +
           (m.matrix[3][0] * p.w);

  temp.y = (m.matrix[0][1] * p.x) +
           (m.matrix[1][1] * p.y) + 
           (m.matrix[2][1] * p.z) +
           (m.matrix[3][1] * p.w);

  temp.z = (m.matrix[0][2] * p.x) +
           (m.matrix[1][2] * p.y) +
           (m.matrix[2][2] * p.z) +
           (m.matrix[3][2] * p.w);

  temp.w = (m.matrix[0][3] * p.x) +
           (m.matrix[1][3] * p.y) +
           (m.matrix[2][3] * p.z) +
           (m.matrix[3][3] * p.w);

  return temp;
}//end multiply(rd_xform, pointh)

rd_xform rd_xform::translate(const rd_xform& m, double tx, double ty, double tz)
{
  rd_xform trans = rd_xform::identity();
  trans.matrix[3][0] = tx;
  trans.matrix[3][1] = ty;
  trans.matrix[3][2] = tz;

  return rd_xform::multiply(m, trans);
}//end translate

rd_xform rd_xform::scale(const rd_xform& m, double sx, double sy, double sz)
{
  rd_xform s = rd_xform::identity();
  s.matrix[0][0] = sx;
  s.matrix[1][1] = sy;
  s.matrix[2][2] = sz;

  return rd_xform::multiply(m, s);
}//end scale

rd_xform rd_xform::rotate_xy(const rd_xform& m, double theta)
{
  // Convert theta to radians for trig functions
  theta *= PI / 180.0;

  rd_xform rot_xy = rd_xform::identity();
  rot_xy.matrix[0][0] = cos(theta);
  rot_xy.matrix[1][0] = -1 * sin(theta);
  rot_xy.matrix[0][1] = sin(theta);
  rot_xy.matrix[1][1] = cos(theta);

  return rd_xform::multiply(m, rot_xy);
}//end rotate_xy

rd_xform rd_xform::rotate_yz(const rd_xform& m, double theta)
{
  // Convert theta to radians for trig functions
  theta *= PI / 180.0;

  rd_xform rot_yz = rd_xform::identity();
  rot_yz.matrix[1][1] = cos(theta);
  rot_yz.matrix[2][1] = -1 * sin(theta);
  rot_yz.matrix[1][2] = sin(theta);
  rot_yz.matrix[2][2] = cos(theta);

  return rd_xform::multiply(m, rot_yz);
}//end rotate_yz

rd_xform rd_xform::rotate_zx(const rd_xform& m, double theta)
{
  // Convert theta to radians for trig functions
  theta *= PI / 180.0;

  rd_xform rot_zx = rd_xform::identity();
  rot_zx.matrix[0][0] = cos(theta);
  rot_zx.matrix[2][0] = sin(theta);
  rot_zx.matrix[0][2] = -1 * sin(theta);
  rot_zx.matrix[2][2] = cos(theta);

  return rd_xform::multiply(m, rot_zx);
}//end rotate_zx

rd_xform rd_xform::world_to_camera(const point& eye, const point& at, const vector& up)
{
  vector A = point::subtract(at, eye);
  vector::normalize(A);
  vector V = vector::cross(A, up);
  vector::normalize(V);
  vector U = vector::cross(V, A);

  // Build the transformation matrix:
  // Vx Vy Vz 0
  // Ux Uy Uz 0
  // Ax Ay Az 0
  //  0  0  0 1
  rd_xform AVU = rd_xform::identity();
  AVU.matrix[0][0] = V.x;
  AVU.matrix[1][0] = V.y;
  AVU.matrix[2][0] = V.z;
  AVU.matrix[0][1] = U.x;
  AVU.matrix[1][1] = U.y;
  AVU.matrix[2][1] = U.z;
  AVU.matrix[0][2] = A.x;
  AVU.matrix[1][2] = A.y;
  AVU.matrix[2][2] = A.z;

  return rd_xform::translate(AVU, -1 * eye.x, -1 * eye.y, -1 * eye.z);
}//end world_to_camera

rd_xform rd_xform::camera_to_clip(double fov, double near, double far, double aspect)
{
  // Convert fov to radians for trig functions
  fov *= PI / 180.0;

  rd_xform toClip = rd_xform::identity();
  toClip.matrix[0][0] = 1 / (aspect * tan(fov / 2.0));
  toClip.matrix[1][1] = 1 / tan(fov / 2.0);
  toClip.matrix[2][2] = far / (far - near);
  toClip.matrix[3][2] = (-1 * far * near) / (far - near);
  toClip.matrix[2][3] = 1;
  toClip.matrix[3][3] = 0;

  rd_xform trans = rd_xform::translate(rd_xform::identity(), 1, 1, 0);
  rd_xform s = rd_xform::scale(rd_xform::identity(), 0.5, 0.5, 1);

  toClip = rd_xform::multiply(trans, toClip);
  toClip = rd_xform::multiply(s, toClip);

  return toClip;
}//end camera_to_clip

rd_xform rd_xform::clip_to_device(int width, int height)
{
  const double epsilon = 0.999;

  rd_xform toDevice = rd_xform::identity();
  toDevice.matrix[0][0] = width - epsilon;
  toDevice.matrix[1][1] = -1 * (height - epsilon);
  toDevice.matrix[3][1] = height - epsilon;

  return toDevice;
}//end clip_to_device
