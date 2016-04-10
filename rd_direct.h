#ifndef RD_ENGINE_DIRECT_H
#define RD_ENGINE_DIRECT_H

#include "rd_enginebase.h"

// This is a rendering engine that renders directly to the image buffer as
// primitives come in.  A depth buffer is obviously needed.  Transparency is
// not properly handled.

class REDirect: public RenderEngine
{
 public:
  int rd_display(const string& name, const string& type, const string& mode);
  int rd_format(int xresolution, int yresolution);

  int rd_world_begin(void);
  int rd_world_end(void);

  int rd_frame_begin(int frame_no);
  int rd_frame_end(void);

  int rd_camera_eye(const float eyepoint[3]);
  int rd_camera_at(const float atpoint[3]);
  int rd_camera_up(const float up[3]);
  int rd_camera_fov(float fov);
  int rd_clipping(float znear, float zfar);

  int rd_translate(const float offset[3]);
  int rd_scale(const float scale_factor[3]);
  int rd_rotate_xy(float angle);
  int rd_rotate_yz(float angle);
  int rd_rotate_zx(float angle);

  int rd_xform_push(void);
  int rd_xform_pop(void);

  int rd_cone(float height, float radius, float thetamax);
  int rd_cube(void);
  int rd_cylinder(float radius, float zmin, float zmax, float thetamax);
  int rd_disk(float height, float radius, float thetamax);
  int rd_sphere(float radius, float zmin, float zmax, float thetamax);

  int rd_background(const float color[]);
  int rd_color(const float color[]);

  int rd_circle(const float center[3], float radius);
  int rd_line(const float start[3], const float end[3]);
  int rd_point(const float p[3]);
  int rd_pointset(const string & vertex_type, int nvertex, const float * vertex);
  int rd_polyset(const string & vertex_type, int nvertex, const float * vertex, int nface, const int * face);

  int rd_fill(const float seed_point[3]);
};

#endif /* RD_ENGINE_DIRECT_H */
