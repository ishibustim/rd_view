// <iostream> included for debugging
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>
#include <stack>
#include "attr_point.h"
#include "clip_bounds.h"
#include "color.h"
#include "edge.h"
#include "point.h"
#include "pointh.h"
#include "rd_direct.h"
#include "rd_display.h"
#include "rd_error.h"
#include "rd_xform.h"
#include "vector.h"

using std::cerr;
using std::clog;
using std::endl;

using std::abs;
using std::min;
using std::max;
using std::ceil;
using std::string;
using std::stack;

bool vertex_color = false;

double** zBuffer = NULL;
edge**   edgeTable = NULL;

const bool MOVE = false;
const bool DRAW = true;
pointh current_line_pointh;

double near_clip = 1.0;
double far_clip = 1.0e+09;

point cam_eye(0, 0, 0);
point cam_at(0, 0, -1);
vector cam_up(0, 1, 0);
double cam_fov = 90;

stack<rd_xform> rd_xform_stack;

rd_xform current_xform;
rd_xform world_to_clip_xform;
rd_xform clip_to_device_xform;

static color rd_direct_color(1, 1, 1);
static color rd_direct_bgColor(0, 0, 0);
static int rd_direct_frameNumber;

double PI = 3.14159265;
int NUM_DIVISIONS = 20;

// Flood Fill functions
void find_span(int& x_start, int& x_end, int y, color& seed_color);
void fff4(int x_start, int x_end, int y, color& seed_color, color& fill_color);

// Pipelines
void point_pipeline(const pointh& p);
void line_pipeline(const pointh& p, bool draw_flag);
void poly_pipeline(attr_point& p, bool draw_flag);

// Clipping
void line_clip(const pointh& p);
int  poly_clip(int n_vertex, attr_point* vertex_list, attr_point* clipped_list);

// Poly Pipeline helpers
void clip_point(attr_point& p, int bound, attr_point* first, attr_point* last, bool* stage_seen, int n_vertex, attr_point* vertex_list, int& n_clipped, attr_point* clipped_list);
void clip_last_point(attr_point* first, attr_point* last, bool* stage_seen, int n_vertex, attr_point* vertex_list, int& n_clipped, attr_point* clipped_list);
bool inside(attr_point& p, int bound);
attr_point intersect(attr_point& p, attr_point& last, int bound);
bool cross(attr_point& p1, attr_point& p2, int bound);

// Pixel Drawing
void display_pixel(int x, int y, double z, float color[3]);

// Polygon Scan Conversion
int minScanLine, maxScanLine;
void scan_convert(attr_point* clipped_list, int n_vertex);
bool buildEdgeList(attr_point* clipped_list, int n_vertex);
void makeEdgeRec(const attr_point& upper, const attr_point& lower);
void addActiveList(int scanline, edge* &aet);
void insertEdge(edge* &list, edge* e);
void edgeFill(int scanline, edge* &aet);
void updateAET(int scanline, edge* &aet);
void resortAET(edge* &aet);
void deleteAfter(edge* &e);

int REDirect::rd_display(const string& name, const string& type, const string& mode)
{
  return RD_OK;
}//end rd_display

int REDirect::rd_format(int xresolution, int yresolution)
{
  return RD_OK;
}//end rd_format

int REDirect::rd_world_begin(void)
{
  current_xform = rd_xform::identity();

  rd_xform world2cam = rd_xform::world_to_camera(cam_eye, cam_at, cam_up);
  rd_xform cam2clip  = rd_xform::camera_to_clip(cam_fov, near_clip, far_clip, (double)display_xSize / (double)display_ySize);
  world_to_clip_xform = rd_xform::multiply(cam2clip, world2cam);

  clip_to_device_xform = rd_xform::clip_to_device((double)display_xSize, (double)display_ySize);
  rd_disp_init_frame(rd_direct_frameNumber);

  if(zBuffer == NULL)
  {
    zBuffer = new double*[display_xSize];
    for(int i = 0; i < display_xSize; i++)
    {
      zBuffer[i] = new double[display_ySize];
    }//end for
  }//end if

  if(edgeTable == NULL)
  {
    edgeTable = new edge*[display_ySize];
    for(int i = 0; i < display_ySize; i++)
    {
      edgeTable[i] = 0;
    }//end for
  }//end if

  for(int i = 0; i < display_xSize; i++)
  {
    for(int j = 0; j < display_ySize; j++)
    {
      zBuffer[i][j] = 1;
    }//end for
  }//end for

  return RD_OK;
}//end rd_world_begin

int REDirect::rd_world_end(void)
{
  rd_disp_end_frame();
  return RD_OK;
}//end rd_world_end

int REDirect::rd_frame_begin(int frame_no)
{
  rd_direct_frameNumber = frame_no;
  return RD_OK;
}//end rd_frame_begin

int REDirect::rd_frame_end(void)
{
  rd_disp_end_frame();
  return RD_OK;
}//end rd_frame_end

int REDirect::rd_render_cleanup(void)
{
  for(int i = 0; i < display_xSize; i++)
  {
    delete zBuffer[i];
  }//end for
  delete zBuffer;
  zBuffer = NULL;

  for(int i = 0; i < display_ySize; i++)
  {
    delete edgeTable[i];
  }//end for
  delete edgeTable;
  edgeTable = NULL;

  return RD_OK;
}//end rd_render_cleanup

int REDirect::rd_camera_eye(const float eyepoint[3])
{
  cam_eye.x = eyepoint[0];
  cam_eye.y = eyepoint[1];
  cam_eye.z = eyepoint[2];

  return RD_OK;
}//end rd_camera_eye

int REDirect::rd_camera_at(const float atpoint[3])
{
  cam_at.x = atpoint[0];
  cam_at.y = atpoint[1];
  cam_at.z = atpoint[2];

  return RD_OK;
}//end rd_camera_at

int REDirect::rd_camera_up(const float up[3])
{
  cam_up.x = up[0];
  cam_up.y = up[1];
  cam_up.z = up[2];

  return RD_OK;
}//end rd_camera_up

int REDirect::rd_camera_fov(float fov)
{
  cam_fov = fov;

  return RD_OK;
}//end rd_camera_fov

int REDirect::rd_clipping(float znear, float zfar)
{
  near_clip = znear;
  far_clip = zfar;

  return RD_OK;
}//end rd_clipping

int REDirect::rd_translate(const float offset[3])
{
  current_xform = rd_xform::translate(current_xform, offset[0], offset[1], offset[2]);

  return RD_OK;
}//end rd_translate

int REDirect::rd_scale(const float scale_factor[3])
{
  current_xform = rd_xform::scale(current_xform, scale_factor[0], scale_factor[1], scale_factor[2]);

  return RD_OK;
}//end rd_scale

int REDirect::rd_rotate_xy(float angle)
{
  current_xform = rd_xform::rotate_xy(current_xform, angle);

  return RD_OK;
}//end rd_rotate_xy

int REDirect::rd_rotate_yz(float angle)
{
  current_xform = rd_xform::rotate_yz(current_xform, angle);

  return RD_OK;
}//end rd_rotate_yz

int REDirect::rd_rotate_zx(float angle)
{
  current_xform = rd_xform::rotate_zx(current_xform, angle);

  return RD_OK;
}//end rd_rotate_zx

int REDirect::rd_xform_push(void)
{
  rd_xform_stack.push(current_xform);

  return RD_OK;
}//end rd_xform_push

int REDirect::rd_xform_pop(void)
{
  current_xform = rd_xform_stack.top();
  rd_xform_stack.pop();

  return RD_OK;
}//end rd_xform_pop

int REDirect::rd_cone(float height, float radius, float thetamax)
{
  for(int theta = 0; theta < NUM_DIVISIONS; theta++)
  {
    // Find theta in radians (based on step / num_divisions)
    double thetaRadians = ((theta / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Find next theta in radians
    double thetaPlusOneRadians = (((theta + 1) / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Create the four corners of each face
    attr_point one;
    one.coord[0] = radius * cos(thetaRadians);
    one.coord[1] = radius * sin(thetaRadians);
    one.coord[2] = 0;
    one.coord[3] = 1;

    attr_point two;
    two.coord[0] = radius * cos(thetaPlusOneRadians);
    two.coord[1] = radius * sin(thetaPlusOneRadians);
    two.coord[2] = 0;
    two.coord[3] = 1;

    attr_point three;
    three.coord[0] = 0;
    three.coord[1] = 0;
    three.coord[2] = height;
    three.coord[3] = 1;

    attr_point four;
    four.coord[0] = 0;
    four.coord[1] = 0;
    four.coord[2] = height;
    four.coord[3] = 1;

    // Draw the face
    poly_pipeline(one, MOVE);
    poly_pipeline(two, MOVE);
    poly_pipeline(three, MOVE);
    poly_pipeline(four, DRAW);
  }//end for

  return RD_OK;
}//end rd_cone

int REDirect::rd_cube(void)
{
  attr_point vertices[8];

  vertices[0].coord[0] = -1;
  vertices[0].coord[1] = -1;
  vertices[0].coord[2] = -1;
  vertices[0].coord[3] =  1;

  vertices[1].coord[0] =  1;
  vertices[1].coord[1] = -1;
  vertices[1].coord[2] = -1;
  vertices[1].coord[3] =  1;

  vertices[2].coord[0] =  1;
  vertices[2].coord[1] = -1;
  vertices[2].coord[2] =  1;
  vertices[2].coord[3] =  1;

  vertices[3].coord[0] = -1;
  vertices[3].coord[1] = -1;
  vertices[3].coord[2] =  1;
  vertices[3].coord[3] =  1;

  vertices[4].coord[0] = -1;
  vertices[4].coord[1] =  1;
  vertices[4].coord[2] = -1;
  vertices[4].coord[3] =  1;

  vertices[5].coord[0] =  1;
  vertices[5].coord[1] =  1;
  vertices[5].coord[2] = -1;
  vertices[5].coord[3] =  1;

  vertices[6].coord[0] =  1;
  vertices[6].coord[1] =  1;
  vertices[6].coord[2] =  1;
  vertices[6].coord[3] =  1;

  vertices[7].coord[0] = -1;
  vertices[7].coord[1] =  1;
  vertices[7].coord[2] =  1;
  vertices[7].coord[3] =  1;

  poly_pipeline(vertices[0], MOVE);
  poly_pipeline(vertices[1], MOVE);
  poly_pipeline(vertices[2], MOVE);
  poly_pipeline(vertices[3], DRAW);

  poly_pipeline(vertices[1], MOVE);
  poly_pipeline(vertices[5], MOVE);
  poly_pipeline(vertices[6], MOVE);
  poly_pipeline(vertices[2], DRAW);

  poly_pipeline(vertices[4], MOVE);
  poly_pipeline(vertices[0], MOVE);
  poly_pipeline(vertices[3], MOVE);
  poly_pipeline(vertices[7], DRAW);

  poly_pipeline(vertices[4], MOVE);
  poly_pipeline(vertices[7], MOVE);
  poly_pipeline(vertices[6], MOVE);
  poly_pipeline(vertices[5], DRAW);

  poly_pipeline(vertices[0], MOVE);
  poly_pipeline(vertices[4], MOVE);
  poly_pipeline(vertices[5], MOVE);
  poly_pipeline(vertices[1], DRAW);

  poly_pipeline(vertices[3], MOVE);
  poly_pipeline(vertices[2], MOVE);
  poly_pipeline(vertices[6], MOVE);
  poly_pipeline(vertices[7], DRAW);

  return RD_OK;
}//end rd_cube

int REDirect::rd_cylinder(float radius, float zmin, float zmax, float thetamax)
{
  for(int theta = 0; theta < NUM_DIVISIONS; theta++)
  {
    // Find theta in radians (based on step / num_divisions)
    double thetaRadians = ((theta / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Find next theta in radians
    double thetaPlusOneRadians = (((theta + 1) / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Create the four corners of each face
    double x        = radius * cos(thetaRadians);
    double y        = radius * sin(thetaRadians);
    double xPlusOne = radius * cos(thetaPlusOneRadians);
    double yPlusOne = radius * sin(thetaPlusOneRadians);

    attr_point one;
    one.coord[0] = x;
    one.coord[1] = y;
    one.coord[2] = zmin;
    one.coord[3] = 1;

    attr_point two;
    two.coord[0] = xPlusOne;
    two.coord[1] = yPlusOne;
    two.coord[2] = zmin;
    two.coord[3] = 1;

    attr_point three;
    three.coord[0] = xPlusOne;
    three.coord[1] = yPlusOne;
    three.coord[2] = zmax;
    three.coord[3] = 1;

    attr_point four;
    four.coord[0] = x;
    four.coord[1] = y;
    four.coord[2] = zmax;
    four.coord[3] = 1;

    // Draw the face
    poly_pipeline(one, MOVE);
    poly_pipeline(two, MOVE);
    poly_pipeline(three, MOVE);
    poly_pipeline(four, DRAW);
  }//end for

  return RD_OK;
}//end rd_cylinder

int REDirect::rd_disk(float height, float radius, float thetamax)
{
  for(int theta = 0; theta < NUM_DIVISIONS; theta++)
  {
    // Find theta in radians (based on step / num_divisions)
    double thetaRadians = ((theta / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Find next theta in radians
    double thetaPlusOneRadians = (((theta + 1) / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

    // Create the four corners of each face
    double x        = radius * cos(thetaRadians);
    double y        = radius * sin(thetaRadians);
    double xPlusOne = radius * cos(thetaPlusOneRadians);
    double yPlusOne = radius * sin(thetaPlusOneRadians);

    attr_point one;
    one.coord[0] = x;
    one.coord[1] = y;
    one.coord[2] = height;
    one.coord[3] = 1;

    attr_point two;
    two.coord[0] = xPlusOne;
    two.coord[1] = yPlusOne;
    two.coord[2] = height;
    two.coord[3] = 1;

    attr_point three;
    three.coord[0] = 0;
    three.coord[1] = 0;
    three.coord[2] = height;
    three.coord[3] = 1;

    attr_point four;
    four.coord[0] = 0;
    four.coord[1] = 0;
    four.coord[2] = height;
    four.coord[3] = 1;

    // Draw the face
    poly_pipeline(one, MOVE);
    poly_pipeline(two, MOVE);
    poly_pipeline(three, MOVE);
    poly_pipeline(four, DRAW);
  }//end for

  return RD_OK;
}//end rd_disk

int REDirect::rd_sphere(float radius, float zmin, float zmax, float thetamax)
{
  for(int phi = 0; phi < NUM_DIVISIONS / 2; phi++)
  {
    // Find phi in radians
    // (Note: the -0.5 is to change the range of phi from -90 to 90 rather than 0 to 180)
    double phiRadians = (((phi / (NUM_DIVISIONS / 2.0)) - 0.5) * 180) * (PI / 180.0);
    double radiusPrime = radius * cos(phiRadians);

    // Find next phi in radians
    double phiPlusOneRadians = ((((phi + 1) / (NUM_DIVISIONS / 2.0)) - 0.5) * 180) * (PI / 180.0);
    double radiusPrimePlusOne = radius * cos(phiPlusOneRadians);

    for(int theta = 0; theta < NUM_DIVISIONS; theta++)
    {
      // Find theta in radians (based on angle / num_divisions)
      double thetaRadians = ((theta / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

      // Find next theta in radians
      double thetaPlusOneRadians = (((theta + 1) / (double)NUM_DIVISIONS) * thetamax) * (PI / 180.0);

      // Create the four corners of each face
      attr_point one;
      one.coord[0] = radiusPrime * cos(thetaRadians);
      one.coord[1] = radiusPrime * sin(thetaRadians);
      one.coord[2] = radius * sin(phiRadians);
      one.coord[3] = 1;

      attr_point two;
      two.coord[0] = radiusPrime * cos(thetaPlusOneRadians);
      two.coord[1] = radiusPrime * sin(thetaPlusOneRadians);
      two.coord[2] = radius * sin(phiRadians);
      two.coord[3] = 1;

      attr_point three;
      three.coord[0] = radiusPrimePlusOne * cos(thetaPlusOneRadians);
      three.coord[1] = radiusPrimePlusOne * sin(thetaPlusOneRadians);
      three.coord[2] = radius * sin(phiPlusOneRadians);
      three.coord[3] = 1;

      attr_point four;
      four.coord[0] = radiusPrimePlusOne * cos(thetaRadians);
      four.coord[1] = radiusPrimePlusOne * sin(thetaRadians);
      four.coord[2] = radius * sin(phiPlusOneRadians);
      four.coord[3] = 1;

      // Draw the face
      poly_pipeline(one, MOVE);
      poly_pipeline(two, MOVE);
      poly_pipeline(three, MOVE);
      poly_pipeline(four, DRAW);
    }//end for
  }//end for

  return RD_OK;
}//end rd_sphere

int REDirect::rd_background(const float color[])
{
  rd_direct_bgColor.setColor(color[0], color[1], color[2]);
  rd_set_background(color);
  return RD_OK;
}//end rd_background

int REDirect::rd_color(const float color[])
{
  rd_direct_color.setColor(color[0], color[1], color[2]);
  return RD_OK;
}//end rd_color

int REDirect::rd_circle(const float center[3], float radius)
{
  int p = 1 - radius;
  int y = radius;

  for(int x = 0; x <= y; x++)
  {
    display_pixel( x + center[0],  y + center[1], center[2], rd_direct_color.getRGB());
    display_pixel( y + center[0],  x + center[1], center[2], rd_direct_color.getRGB());
    display_pixel(-x + center[0],  y + center[1], center[2], rd_direct_color.getRGB());
    display_pixel(-y + center[0],  x + center[1], center[2], rd_direct_color.getRGB());
    display_pixel( x + center[0], -y + center[1], center[2], rd_direct_color.getRGB());
    display_pixel( y + center[0], -x + center[1], center[2], rd_direct_color.getRGB());
    display_pixel(-x + center[0], -y + center[1], center[2], rd_direct_color.getRGB());
    display_pixel(-y + center[0], -x + center[1], center[2], rd_direct_color.getRGB());
    if(p > 0)
    {
      p += 2*x - 2*y + 5;
      y--;
    }//end if
    else
    {
      p += 2*(x + 1) + 1;
    }//end else
  }//end for
  return RD_OK;
}//end rd_circle

int REDirect::rd_line(const float start[3], const float end[3])
{
  pointh endpoint1(start[0], start[1], start[2], 1);
  pointh endpoint2(end[0], end[1], end[2], 1);

  line_pipeline(endpoint1, MOVE);
  line_pipeline(endpoint2, DRAW);

  return RD_OK;
}//end rd_line

int REDirect::rd_point(const float p[3])
{
  pointh newP(p[0], p[1], p[2], 1);
  point_pipeline(newP);
  return RD_OK;
}//end rd_point

int REDirect::rd_pointset(const string & vertex_type, int nvertex, const float * vertex)
{
  int numValues = get_vertex_size(vertex_type);

  for(int i = 0; i < nvertex; i++)
  {
    pointh p(vertex[(i * numValues)], vertex[(i * numValues)+1], vertex[(i * numValues)+2], 1);
    point_pipeline(p);
  }//end for

  return RD_OK;
}//end rd_pointset

int REDirect::rd_polyset(const string & vertex_type, int nvertex, const float * vertex, int nface, const int * face)
{
  int numValues = get_vertex_size(vertex_type);
  bool flag = MOVE;

  int counter = 0;
  for(int i = 0; counter < nface; i++)
  {
    if(face[i] != -1)
    {
      attr_point temp;
      temp.coord[0] = vertex[(face[i] * numValues)];
      temp.coord[1] = vertex[(face[i] * numValues) + 1];
      temp.coord[2] = vertex[(face[i] * numValues) + 2];
      temp.coord[3] = 1;
      if(face[i + 1] != -1)
        flag = MOVE;
      else
        flag = DRAW;
      poly_pipeline(temp, flag);
    }//end if
    else
    {
      counter++;
    }//end else
  }//end for
  return RD_OK;
}//end rd_polyset

int REDirect::rd_fill(const float seed_point[3])
{
  float tempColorArray[3];
  rd_read_pixel((int)seed_point[0], (int)seed_point[1], tempColorArray);
  color seed_color(tempColorArray[0], tempColorArray[1], tempColorArray[2]);

  int x_start = seed_point[0];
  int x_end = seed_point[0];

  find_span(x_start, x_end, (int)seed_point[1], seed_color);
  if(x_start < x_end)
    fff4(x_start, x_end, (int)seed_point[1], seed_color, rd_direct_color);
  return RD_OK;
}//end rd_fill

void find_span(int& x_start, int& x_end, int y, color& seed_color)
{
  float tempColorArray[3];
  rd_read_pixel(x_start, y, tempColorArray);
  color tempColor(tempColorArray[0], tempColorArray[1], tempColorArray[2]);
  if(!(tempColor == seed_color))
    return;
  while(tempColor == seed_color && x_start >= 0)
  {
    x_start--;
    if(x_start >= 0)
    {
      rd_read_pixel(x_start, y, tempColorArray);
      tempColor.setColor(tempColorArray[0], tempColorArray[1], tempColorArray[2]);
    }//end if
  }//end while
  // The loop will exit with x_start on a non-fill pixel
  // Increment x_start to get the correct starting location
  x_start++;

  // Since x_end is one pixel past the last fill pixel,
  // start one pixel past the initial pixel
  x_end++;
  if(x_end < display_xSize)
  {
    rd_read_pixel(x_end, y, tempColorArray);
    tempColor.setColor(tempColorArray[0], tempColorArray[1], tempColorArray[2]);
  }//end if
  while(tempColor == seed_color && x_end < display_xSize)
  {
    x_end++;
    if(x_end < display_xSize)
    {
      rd_read_pixel(x_end, y, tempColorArray);
      tempColor.setColor(tempColorArray[0], tempColorArray[1], tempColorArray[2]);
    }//end if
  }//end while
}//end find_span

// fff4 = Fast Flood Fill 4-Connected Neighbors
void fff4(int x_start, int x_end, int y, color& seed_color, color& fill_color)
{
  int nextYplus = x_start;
  int nextYminus = x_start;
  int new_x_start, new_x_end;
  // Loop through valid portion of row
  for(int x = x_start; x < x_end; x++)
  {
    // Color each pixel
    rd_write_pixel(x, y, fill_color.getRGB());
  }//end for

  for(int x = x_start; x < x_end; x++)
  {
    new_x_start = x;
    new_x_end = x;

    if(y + 1 != display_ySize && x == nextYplus)
    {
      // Find valid span at y+1
      find_span(new_x_start, new_x_end, y + 1, seed_color);
      //nextYplus = new_x_end + 1;
      nextYplus++;
      // Color valid span at y+1
      if(new_x_start < new_x_end)
        fff4(new_x_start, new_x_end, y + 1, seed_color, fill_color);
    }//end if

    if(y != 0 && x == nextYminus)
    {
      // Find valid span at y-1
      new_x_start = new_x_end = x;
      find_span(new_x_start, new_x_end, y - 1, seed_color);
      //nextYminus = new_x_end + 1;
      nextYminus++;
      // Color valid span at y-1
      if(new_x_start < new_x_end)
        fff4(new_x_start, new_x_end, y - 1, seed_color, fill_color);
    }//end if
  }//end for
}//end fff4

void point_pipeline(const pointh& p)
{
  // Copy pointh
  pointh newP(p.x, p.y, p.z, p.w);

  newP = rd_xform::multiply(current_xform, newP);
  newP = rd_xform::multiply(world_to_clip_xform, newP);

  // Find boundary coordinates
  double wx, wy, wz;
  wx = newP.w - newP.x;
  wy = newP.w - newP.y;
  wz = newP.w - newP.z;

  // Clip using boundary coordinates
  if(newP.x >= 0 && wx >= 0 &&
     newP.y >= 0 && wy >= 0 &&
     newP.z >= 0 && wz >= 0)
  {
    newP = rd_xform::multiply(clip_to_device_xform, newP);
    
    newP.x /= newP.w;
    newP.y /= newP.w;
    newP.z /= newP.w;
    newP.w /= newP.w;

    display_pixel(newP.x, newP.y, newP.z, rd_direct_color.getRGB());
  }//end if
}//end point_pipeline

void line_pipeline(const pointh& p, bool draw_flag)
{
  if(!draw_flag)
  {
    // Store beginning endpoint in clip coordinates
    current_line_pointh = rd_xform::multiply(current_xform, p);
    current_line_pointh = rd_xform::multiply(world_to_clip_xform, current_line_pointh);
    //current_line_pointh = rd_xform::multiply(clip_to_device_xform, current_line_pointh);
  }//end if
  else
  {
    pointh next_line_pointh;

    // Transform next endpoint to clip coordinates
    next_line_pointh = rd_xform::multiply(current_xform, p);
    next_line_pointh = rd_xform::multiply(world_to_clip_xform, next_line_pointh);
    //next_line_pointh = rd_xform::multiply(clip_to_device_xform, next_line_pointh);

    line_clip(next_line_pointh);
  }//end else
}//end line_pipeline

void poly_pipeline(attr_point& p, bool draw_flag)
{
  pointh geom, norm;
  attr_point ap;

  // Copy the input vertex
  for(int i = 0; i < ATTR_SIZE; i++)
  {
    ap.coord[i] = p.coord[i];
  }//end for

  const int MAX_VERTEX_LIST_SIZE = 50;
  static attr_point vertex_list[MAX_VERTEX_LIST_SIZE];
  static attr_point clipped_list[MAX_VERTEX_LIST_SIZE * 2];
  static int n_vertex = 0;
  static int n_clipped = 0;

  geom.x = ap.coord[0];
  geom.y = ap.coord[1];
  geom.z = ap.coord[2];
  geom.w = ap.coord[3];

  // Transform by current transform and transform to clip coords
  geom = rd_xform::multiply(current_xform, geom);
  geom = rd_xform::multiply(world_to_clip_xform, geom);

  ap.coord[0] = geom.x;
  ap.coord[1] = geom.y;
  ap.coord[2] = geom.z;
  ap.coord[3] = geom.w;

  // Store in vertex list
  if(n_vertex == MAX_VERTEX_LIST_SIZE)
  {
    cerr << "ERROR: poly_pipeline: Overflow" << endl;
    return; // Overflow
  }//end if

  vertex_list[n_vertex] = ap;
  n_vertex++;

  if(draw_flag == MOVE) // Move along to the next vertex
    return;

  n_clipped = poly_clip(n_vertex, vertex_list, clipped_list);

  if(n_clipped != 0)
  {
    // Draw the polygon
    // Pre-process vertex list
    for(int i = 0; i < n_clipped; i++)
    {
      pointh dev;
      // Convert geometry to device coordinates
      dev.x = clipped_list[i].coord[0];
      dev.y = clipped_list[i].coord[1];
      dev.z = clipped_list[i].coord[2];
      dev.w = clipped_list[i].coord[3];

      dev = rd_xform::multiply(clip_to_device_xform, dev);

      clipped_list[i].coord[0] = dev.x - 0.5;
      clipped_list[i].coord[1] = dev.y - 0.5;
      clipped_list[i].coord[2] = dev.z;
      clipped_list[i].coord[3] = dev.w;

      // Divide geometry by W
      clipped_list[i].coord[0] /= clipped_list[i].coord[3];
      clipped_list[i].coord[1] /= clipped_list[i].coord[3];
      clipped_list[i].coord[2] /= clipped_list[i].coord[3];
    }//end for

    scan_convert(clipped_list, n_clipped);
  }//end if

  // Reset structures for next polygon
  n_vertex = 0;
  n_clipped = 0;
}//end poly_pipeline

void line_clip(const pointh& p)
{
  pointh next_line_pointh(p.x, p.y, p.z, p.w);

  double BC1[6] = { current_line_pointh.x,
		    current_line_pointh.w - current_line_pointh.x, 
		    current_line_pointh.y,
		    current_line_pointh.w - current_line_pointh.y,
		    current_line_pointh.z,
		    current_line_pointh.w - current_line_pointh.z };

  double BC2[6] = { next_line_pointh.x,
		    next_line_pointh.w - next_line_pointh.x,
		    next_line_pointh.y,
		    next_line_pointh.w - next_line_pointh.y,
		    next_line_pointh.z,
		    next_line_pointh.w - next_line_pointh.z };

  int kode1 = 0;
  int kode2 = 0;
  int mask = 1;

  for(int i = 5; i >= 0; i--)
  {
    if(BC1[i] < 0.0)
    {
      kode1 = kode1 | mask;
    }//end if
    if(BC2[i] < 0.0)
    {
      kode2 = kode2 | mask;
    }//end if

    mask = mask << 1;
  }//end for

  mask = 1;
  int kode = kode1 | kode2;

  int trivRej = kode1 & kode2;
  int trivAcc = kode1 | kode2;

  double alphaMin = 0;
  double alphaMax = 1;

  // For each boundary
  for(int i = 0; i < 6 && !trivAcc && !trivRej; i++)
  {
    // If the line does not cross
    // this boundary, go to the next
    if(kode & mask)
    {
      double alpha = BC1[i] / (BC1[i] - BC2[i]);
      if(kode1 & mask)
      {
	alphaMin = max(alphaMin, alpha);
      }//end if
      else
      {
	alphaMax = min(alphaMax, alpha);
      }//end else
      mask <<= 1;
    }//end if
  }//end for

  if(trivRej)
  {
    current_line_pointh = next_line_pointh;
  }//end if

  if(alphaMin < alphaMax && !trivRej)
  {
    pointh newP0(current_line_pointh.x + alphaMin * (next_line_pointh.x - current_line_pointh.x),
		 current_line_pointh.y + alphaMin * (next_line_pointh.y - current_line_pointh.y),
		 current_line_pointh.z + alphaMin * (next_line_pointh.z - current_line_pointh.z),
		 current_line_pointh.w + alphaMin * (next_line_pointh.w - current_line_pointh.w));

    pointh newP1(current_line_pointh.x + alphaMax * (next_line_pointh.x - current_line_pointh.x),
		 current_line_pointh.y + alphaMax * (next_line_pointh.y - current_line_pointh.y),
		 current_line_pointh.z + alphaMax * (next_line_pointh.z - current_line_pointh.z),
		 current_line_pointh.w + alphaMax * (next_line_pointh.w - current_line_pointh.w));

    current_line_pointh = next_line_pointh;

    newP0 = rd_xform::multiply(clip_to_device_xform, newP0);
    newP1 = rd_xform::multiply(clip_to_device_xform, newP1);

    double endpoint1[] = {newP0.x / newP0.w, newP0.y / newP0.w, newP0.z / newP0.w};
    double endpoint2[] = {newP1.x / newP1.w, newP1.y / newP1.w, newP1.z / newP1.w};

    // Begin drawing the line
    double deltaX = (int)endpoint2[0] - (int)endpoint1[0];
    double deltaY = (int)endpoint2[1] - (int)endpoint1[1];
    double deltaZ = (int)endpoint2[2] - (int)endpoint1[2];

    int numsteps;

    // Determine whether to step in X, Y, or Z
    // X
    if(abs(deltaX) >= abs(deltaY) && abs(deltaX) >= abs(deltaZ))
    {
      numsteps = abs((int)endpoint2[0] - (int)endpoint1[0]);
    }//end if
    // Y
    else if(abs(deltaY) > abs(deltaX) && abs(deltaY) > abs(deltaZ))
    {
      numsteps = abs((int)endpoint2[1] - (int)endpoint1[1]);
    }//end else if
    // Z
    else
    {
      numsteps = abs((int)endpoint2[2] - (int)endpoint1[2]);
    }//end else

    double x = (int)endpoint1[0];
    double y = (int)endpoint1[1];
    double z = endpoint1[2];

    for(int i = 0; i < numsteps; i++)
    {
      display_pixel(x, y, z, rd_direct_color.getRGB());
      x += deltaX / numsteps;
      y += deltaY / numsteps;
      z += deltaZ / numsteps;
    }//end for
  }//end if
}//end line_clip

int poly_clip(int n_vertex, attr_point* vertex_list, attr_point* clipped_list)
{
  attr_point first[6];
  attr_point last[6];
  bool stage_seen[6] = {false, false, false, false, false, false};
  int n_clipped = 0;

  for(int i = 0; i < n_vertex; i++)
  {
    clip_point(vertex_list[i], left, first, last, stage_seen, n_vertex, vertex_list, n_clipped, clipped_list);
  }//end for

  clip_last_point(first, last, stage_seen, n_vertex, vertex_list, n_clipped, clipped_list);

  return n_clipped;
}//end poly_clip

void clip_point(attr_point& p, int bound, attr_point* first, attr_point* last, bool* stage_seen, int n_vertex, attr_point* vertex_list, int& n_clipped, attr_point* clipped_list)
{
  // Copy input attr_point
  attr_point ap;
  for(int i = 0; i < ATTR_SIZE; i++)
  {
    ap.coord[i] = p.coord[i];
  }//end for

  // If this is the first time a point has been
  // seen at this stage
  if(!stage_seen[bound])
  {
    first[bound] = p;
    stage_seen[bound] = true;
  }//end if
  else
  {
    // If the edge formed by p and last[bound] crosses
    // the boundary "bound", clip it
    if(cross(ap, last[bound], bound))
    {
      // Find the intersection point
      attr_point ipt = intersect(ap, last[bound], bound);
      if(bound < top)
      {
	clip_point(ipt, bound + 1, first, last, stage_seen, n_vertex, vertex_list, n_clipped, clipped_list);
      }//end if
      else
      {
	clipped_list[n_clipped] = ipt;
	++n_clipped;
      }//end else
    }//end if
  }//end else

  // Save the most recent vertex seen at this stage
  last[bound] = p;

  if(inside(ap, bound))
  {
     if(bound < top)
     {
       clip_point(ap, bound + 1, first, last, stage_seen, n_vertex, vertex_list, n_clipped, clipped_list);
     }//end if
     else
     {
       clipped_list[n_clipped] = ap;
       ++n_clipped;
     }//end else
  }//end if
}//end clip_point

void clip_last_point(attr_point* first, attr_point* last, bool* stage_seen, int n_vertex, attr_point* vertex_list, int& n_clipped, attr_point* clipped_list)
{
  // Loop over each boundary
  for(int bound = 0; bound < 6; bound++)
  {
    if(stage_seen[bound])
    {
      if(cross(first[bound], last[bound], bound))
      {
	attr_point ipt = intersect(last[bound], first[bound], bound);

	if(bound < top)
	{
	  clip_point(ipt, bound + 1, first, last, stage_seen, n_vertex, vertex_list, n_clipped, clipped_list);
	}//end if
	else
	{
	  clipped_list[n_clipped] = ipt;
	  ++n_clipped;
	}//end else
      }//end if
    }//end if
  }//end for
}//end clip_last_point

bool inside(attr_point& p, int bound)
{
  switch(bound)
  {
  case left:
    if(p.coord[0] >= 0)
      return true;
    else
      return false;
  
  case right:
    if(p.coord[3] - p.coord[0] >= 0)
      return true;
    else
      return false;

  case front:
    if(p.coord[1] >= 0)
      return true;
    else
      return false;

  case back:
    if(p.coord[3] - p.coord[1] >= 0)
      return true;
    else
      return false;

  case bottom:
    if(p.coord[2] >= 0)
      return true;
    else
      return false;

  case top:
    if(p.coord[3] - p.coord[2] >= 0)
      return true;
    else
      return false;

    // This should never execute
  default:
    cerr << "inside: !!WARNING!! -- Switch default case executed"  << endl;
    return false;
  }//end switch
}//end inside

attr_point intersect(attr_point& p, attr_point& last, int bound)
{
  double boundary_coord1, boundary_coord2;

  switch(bound)
  {
  case left:
    boundary_coord1 = p.coord[0];
    boundary_coord2 = last.coord[0];
    break;
  case right:
    boundary_coord1 = p.coord[3] - p.coord[0];
    boundary_coord2 = last.coord[3] - last.coord[0];
    break;
  case front:
    boundary_coord1 = p.coord[1];
    boundary_coord2 = last.coord[1];
    break;
  case back:
    boundary_coord1 = p.coord[3] - p.coord[1];
    boundary_coord2 = last.coord[3] - last.coord[1];
    break;
  case bottom:
    boundary_coord1 = p.coord[2];
    boundary_coord2 = last.coord[2];
    break;
  case top:
    boundary_coord1 = p.coord[3] - p.coord[2];
    boundary_coord2 = last.coord[3] - last.coord[2];
    break;

    // This should never execute
  default:
    cerr << "intersect: !!WARNING!! -- Switch default case executed" << endl;
    boundary_coord1 = p.coord[0];
    boundary_coord2 = last.coord[0];
  }//end switch

  // Find alpha to interpolate to
  double alphaMin = 0;
  double alphaMax = 1;
  double alpha = boundary_coord1 / (boundary_coord1 - boundary_coord2);

  if(boundary_coord1 < 0.0)
    alphaMin = max(alphaMin, alpha);
  if(boundary_coord2 < 0.0)
    alphaMax = min(alphaMax, alpha);

  attr_point out;

  if(alphaMin < alphaMax)
  {
    if(alphaMin != 0)
      alpha = alphaMin;
    if(alphaMax != 1)
      alpha = alphaMax;

    for(int i = 0; i < ATTR_SIZE; i++)
    {
      out.coord[i] = p.coord[i] + alpha * (last.coord[i] - p.coord[i]);
    }//end for
  }//end if

  return out;
}//end intersect

bool cross(attr_point& p1, attr_point& p2, int bound)
{
  return inside(p1, bound) != inside(p2, bound);
}//end cross

void display_pixel(int x, int y, double z, float color[3])
{
  if(x >= 0 && x < display_xSize &&
     y >= 0 && y < display_ySize)
  {
    if(z < zBuffer[x][y])
    {
      rd_write_pixel(x, y, color);
      zBuffer[x][y] = z;
    }//end if
  }//end if
}//end display_pixel

void scan_convert(attr_point* clipped_list, int n_vertex)
{
  edge *aet = NULL; // Head of the Active Edge Table (AET)

  // Set bounds of scan conversion
  // minScanLine will be the lowest scanline with a pixel to be drawn
  // maxScanLine will be the highest scanline with a pixel to be drawn
  // This *should* speed up the rendering a bit
  minScanLine = display_ySize;
  maxScanLine = 0;

  if(!buildEdgeList(clipped_list, n_vertex))
    return; // No edges cross a scanline

  // Loop over each scanline
  for(int scanline = minScanLine; scanline <= maxScanLine; scanline++)
  {
    // Copy edgetable at current scanline to Active Edge Table
    // and keep the AET sorted by X coordinates
    addActiveList(scanline, aet);

    if(aet != NULL)
    {
      edgeFill(scanline, aet);
      updateAET(scanline, aet);
      resortAET(aet);
    }//end if
  }//end for
}//end scan_convert

bool buildEdgeList(attr_point* clipped_list, int n_vertex)
{
  // v1 and v2 are indices into the attr_point array
  // v1 is the trailing vertex of the edge
  // v2 is the leading vertex of the edge
  int v1, v2;
  bool scanline_crossed = false;

  v1 = n_vertex - 1; // The last vertex in the polygon
  for(v2 = 0; v2 < n_vertex; v2++)
  {
    // If the endpoints fall on different scanlines
    if(ceil(clipped_list[v1].coord[1]) != ceil(clipped_list[v2].coord[1]))
    {
      scanline_crossed = true;

      if(clipped_list[v1].coord[1] < clipped_list[v2].coord[1])
      {
	makeEdgeRec(clipped_list[v2], clipped_list[v1]);
      }//end if
      else
      {
	makeEdgeRec(clipped_list[v1], clipped_list[v2]);
      }//end else
    }//end if
    // Move to next edge
    v1 = v2;
  }//end for

  return scanline_crossed;
}//end buildEdgeList

void makeEdgeRec(const attr_point& upper, const attr_point& lower)
{
  double dy, factor;
  edge *e;

  dy = upper.coord[1] - lower.coord[1];

  e = new edge();

  // Calculate the edge value increments between scan lines
  for(int i = 0; i < ATTR_SIZE; i++)
  {
    e->inc.coord[i] = (upper.coord[i] - lower.coord[i]) / dy;
  }//end for

  // Edge starts on scanline ceil(lower.y - 0.5)
  // Get the fractional position of the first scanline crossing
  // This is used to interpolate the endpoint to the
  // TRUE scanline values
  factor = ceil(lower.coord[1]) - lower.coord[1];

  // Calculate the starting values for the edge
  for(int i = 0; i < ATTR_SIZE; i++)
  {
    e->p.coord[i] = lower.coord[i] + (factor * e->inc.coord[i]);
  }//end for

  // Find the last scanline for the edge
  e->yLast = ceil(upper.coord[1]) - 1;

  int scanline = ceil(lower.coord[1]);
  if(scanline < minScanLine)
    minScanLine = scanline;
  if(e->yLast > maxScanLine)
    maxScanLine = e->yLast;

  insertEdge(edgeTable[scanline], e);
  delete e;
}//end makeEdgeRec

void addActiveList(int scanline, edge* &aet)
{
  edge *p, *q;

  q = edgeTable[scanline]; // Get the edges starting on this scan line

  while(q)
  {
    p = q->next; // Hold the rest of the list
    insertEdge(aet, q);
    q = p;
  }//end while

  // Keep the edge table clean -- edges have been transfered
  edge* temp = edgeTable[scanline];
  while(temp)
  {
    edge* old = temp;
    temp = temp->next;
    delete old;
  }//end while

  edgeTable[scanline] = NULL;
}//end addActiveList

void insertEdge(edge* &list, edge* e)
{
  // insert at head of list if list is empty
  if(list == NULL)
  {
    edge* newE = new edge(*e);
    newE->next = NULL;
    list = newE;
  }//end if
  // If the new edge should come before the list head, insert at list head
  else if(e->p.coord[0] < list->p.coord[0])
  {
    edge *newE = new edge(*e);
    newE->next = list;
    list = newE;
  }//end else if
  // Find correct insertion point
  else
  {
    edge *p, *q = list;

    // p leads
    p = q->next;

    while(p != 0 && (e->p.coord[0] > p->p.coord[0]))
    {
      // Step to the next edge
      q = p;
      p = p->next;
    }//end while

    // Link the edge into the list after q
    edge *newE = new edge(*e);
    newE->next = p;
    q->next = newE;
  }//end else
}//end insertEdge

void edgeFill(int scanline, edge* &aet)
{
  edge *p1, *p2;

  p1 = aet;
  while(p1 != 0)
  {
    p2 = p1->next; // Get the pair of edges from the AET

    // If the scanline is wider than one pixel
    if(ceil(p1->p.coord[0]) != ceil(p2->p.coord[0]))
    {
      // Calculate the attributed increments along the scanline
      double dx = p2->p.coord[0] - p1->p.coord[0];

      attr_point inc, value;

      for(int i = 0; i < ATTR_SIZE; i++)
      {
	inc.coord[i] = (p2->p.coord[i] - p1->p.coord[i]) / dx;
      }//end for

      // Calculate the starting values for the edge
      double factor = ceil(p1->p.coord[0]) - p1->p.coord[0];

      for(int i = 0; i < ATTR_SIZE; i++)
      {
	value.coord[i] = p1->p.coord[i] + (factor * inc.coord[i]);
      }//end for

      int endx = ceil(p2->p.coord[0]);

      while(value.coord[0] <= endx)
      {
	// Retreive color from current value
	float color[] = {value.coord[ATTR_R], value.coord[ATTR_G], value.coord[ATTR_B]};

	if(vertex_color)
	{
	  display_pixel(value.coord[0], scanline, value.coord[2], color);
	}//end if
	else
	{
	  display_pixel(value.coord[0], scanline, value.coord[2], rd_direct_color.getRGB());
	}//end else

	// Increment the values
	for(int i = 0; i < ATTR_SIZE; i++)
	{
	  value.coord[i] += inc.coord[i];
	}//end for
      }//end while
    }//end if

    p1 = p2->next;

  }//end while
}//end edgeFill

void updateAET(int scanline, edge* &aet)
{
  edge *q = aet;
  edge *p = aet->next;

  // Update all edges other than first
  while(p)
  {
    if(scanline == p->yLast) // This is the last scanline for this edge
    {
      p = p->next;    // Move P along
      deleteAfter(q); // Get rid of old node
    }//end if
    else
    {
      // Update the attribute values
      for(int i = 0; i < ATTR_SIZE; i++)
      {
	p->p.coord[i] += p->inc.coord[i];
      }//end for

      q = p;
      p = p->next;
    }//end else
  }//end while

  // Update first edge
  if(scanline == aet->yLast)
  {
    q = aet;
    aet = aet->next;
    delete q;
  }//end if
  else
  {
    for(int i = 0; i < ATTR_SIZE; i++)
    {
      aet->p.coord[i] += aet->inc.coord[i];
    }//end for
  }//end else
}//end updateAET

void resortAET(edge* &aet)
{
  if(aet != NULL)
  {
    edge *q = aet;
    edge *p = aet->next;

    aet->next = NULL;

    while(p)
    {
      insertEdge(aet, p);
      q = p;
      p = p->next;
      delete q;
    }//end while
  }//end if
}//end resortAET

void deleteAfter(edge* &e)
{
  edge *p = e->next;

  e->next = p->next;
  delete p;
}//end deleteAfter
