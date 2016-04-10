// <iostream> included for debugging
#include <iostream>

#include <algorithm>
#include <cmath>
#include <string>
#include <stack>
#include "color.h"
#include "point.h"
#include "pointh.h"
#include "rd_direct.h"
#include "rd_display.h"
#include "rd_error.h"
#include "rd_xform.h"
#include "vector.h"

using std::clog;
using std::endl;

using std::abs;
using std::min;
using std::max;
using std::string;
using std::stack;

double** zBuffer = NULL;

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

// Clipping
void line_clip(const pointh& p);

// Pixel Drawing
void display_pixel(int x, int y, double z, float color[3]);

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
    pointh one(radius * cos(thetaRadians), radius * sin(thetaRadians), 0, 1);

    pointh two(radius * cos(thetaPlusOneRadians), radius * sin(thetaPlusOneRadians), 0, 1);

    pointh three(0, 0, height, 1);

    pointh four(0, 0, height, 1);

    // Draw the face
    line_pipeline(one, MOVE);
    line_pipeline(two, DRAW);
    line_pipeline(three, DRAW);
    line_pipeline(four, DRAW);
    line_pipeline(one, DRAW);    
  }//end for

  return RD_OK;
}//end rd_cone

int REDirect::rd_cube(void)
{
  pointh verteces[8];
  verteces[0] = pointh(-1, -1, -1, 1);
  verteces[1] = pointh( 1, -1, -1, 1);
  verteces[2] = pointh( 1, -1,  1, 1);
  verteces[3] = pointh(-1, -1,  1, 1);
  verteces[4] = pointh(-1,  1, -1, 1);
  verteces[5] = pointh( 1,  1, -1, 1);
  verteces[6] = pointh( 1,  1,  1, 1);
  verteces[7] = pointh(-1,  1,  1, 1);

  line_pipeline(verteces[0], MOVE);
  line_pipeline(verteces[1], DRAW);
  line_pipeline(verteces[2], DRAW);
  line_pipeline(verteces[3], DRAW);
  line_pipeline(verteces[0], DRAW);

  line_pipeline(verteces[1], MOVE);
  line_pipeline(verteces[5], DRAW);
  line_pipeline(verteces[6], DRAW);
  line_pipeline(verteces[2], DRAW);
  line_pipeline(verteces[1], DRAW);

  line_pipeline(verteces[4], MOVE);
  line_pipeline(verteces[0], DRAW);
  line_pipeline(verteces[3], DRAW);
  line_pipeline(verteces[7], DRAW);
  line_pipeline(verteces[4], DRAW);

  line_pipeline(verteces[4], MOVE);
  line_pipeline(verteces[7], DRAW);
  line_pipeline(verteces[6], DRAW);
  line_pipeline(verteces[5], DRAW);
  line_pipeline(verteces[4], DRAW);

  line_pipeline(verteces[0], MOVE);
  line_pipeline(verteces[4], DRAW);
  line_pipeline(verteces[5], DRAW);
  line_pipeline(verteces[1], DRAW);
  line_pipeline(verteces[0], DRAW);

  line_pipeline(verteces[3], MOVE);
  line_pipeline(verteces[2], DRAW);
  line_pipeline(verteces[6], DRAW);
  line_pipeline(verteces[7], DRAW);
  line_pipeline(verteces[3], DRAW);

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

    pointh one(x, y, zmin, 1);

    pointh two(xPlusOne, yPlusOne, zmin, 1);

    pointh three(xPlusOne, yPlusOne, zmax, 1);

    pointh four(x, y, zmax, 1);

    // Draw the face
    line_pipeline(one, MOVE);
    line_pipeline(two, DRAW);
    line_pipeline(three, DRAW);
    line_pipeline(four, DRAW);
    line_pipeline(one, DRAW);
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

    pointh one(x, y, height, 1);

    pointh two(xPlusOne, yPlusOne, height, 1);

    pointh three(0, 0, height, 1);

    pointh four(0, 0, height, 1);

    // Draw the face
    line_pipeline(one, MOVE);
    line_pipeline(two, DRAW);
    line_pipeline(three, DRAW);
    line_pipeline(four, DRAW);
    line_pipeline(one, DRAW);    
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
      pointh one(radiusPrime * cos(thetaRadians),
		 radiusPrime * sin(thetaRadians),
		 radius * sin(phiRadians), 1);

      pointh two(radiusPrime * cos(thetaPlusOneRadians),
		 radiusPrime * sin(thetaPlusOneRadians),
		 radius * sin(phiRadians), 1);

      pointh three(radiusPrimePlusOne * cos(thetaPlusOneRadians),
		   radiusPrimePlusOne * sin(thetaPlusOneRadians),
		   radius * sin(phiPlusOneRadians), 1);

      pointh four(radiusPrimePlusOne * cos(thetaRadians),
		  radiusPrimePlusOne * sin(thetaRadians),
		  radius * sin(phiPlusOneRadians), 1);

      // Draw the face
      line_pipeline(one, MOVE);
      line_pipeline(two, DRAW);
      line_pipeline(three, DRAW);
      line_pipeline(four, DRAW);
      line_pipeline(one, DRAW);
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
  int faceBegin = 0;
  bool flag = MOVE;

  int counter = 0;
  for(int i = 0; counter < nface; i++)
  {
    if(face[i] == -1)
    {
      pointh temp(vertex[(face[faceBegin] * numValues)],
		  vertex[(face[faceBegin] * numValues) + 1],
		  vertex[(face[faceBegin] * numValues) + 2], 1);
      line_pipeline(temp, flag);

      counter++;
      flag = MOVE;
      faceBegin = i + 1;
    }//end if
    else
    {
      pointh temp(vertex[(face[i] * numValues)], vertex[(face[i] * numValues) + 1], vertex[(face[i] * numValues) + 2], 1);

      line_pipeline(temp, flag);

      if(flag == MOVE)
	flag = DRAW;
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
    double deltaX = endpoint2[0] - endpoint1[0];
    double deltaY = endpoint2[1] - endpoint1[1];
    double deltaZ = endpoint2[2] - endpoint1[2];

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

    double x = endpoint1[0];
    double y = endpoint1[1];
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
