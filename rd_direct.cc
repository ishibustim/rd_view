#include <iostream>

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

using std::string;
using std::stack;

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
  for(int phi = 0; phi < NUM_DIVISIONS; phi++)
  {
    // Find phi in radians
    // (Note: the -0.5 is to change the range of phi from -90 to 90 rather than 0 to 180)
    double phiRadians = (((phi / (double)NUM_DIVISIONS) - 0.5) * 180) * (PI / 180.0);
    double radiusPrime = radius * cos(phiRadians);

    // Find next phi in radians
    double phiPlusOneRadians = ((((phi + 1) / (double)NUM_DIVISIONS) - 0.5) * 180) * (PI / 180.0);
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
    rd_write_pixel( x + center[0],  y + center[1], rd_direct_color.getRGB());
    rd_write_pixel( y + center[0],  x + center[1], rd_direct_color.getRGB());
    rd_write_pixel(-x + center[0],  y + center[1], rd_direct_color.getRGB());
    rd_write_pixel(-y + center[0],  x + center[1], rd_direct_color.getRGB());
    rd_write_pixel( x + center[0], -y + center[1], rd_direct_color.getRGB());
    rd_write_pixel( y + center[0], -x + center[1], rd_direct_color.getRGB());
    rd_write_pixel(-x + center[0], -y + center[1], rd_direct_color.getRGB());
    rd_write_pixel(-y + center[0], -x + center[1], rd_direct_color.getRGB());
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

    rd_write_pixel(newP.x, newP.y, rd_direct_color.getRGB());
  }//end if
}//end point_pipeline

void line_pipeline(const pointh& p, bool draw_flag)
{
  if(!draw_flag)
  {
    // Store beginning endpoint in device coordinates
    current_line_pointh = rd_xform::multiply(current_xform, p);
    current_line_pointh = rd_xform::multiply(world_to_clip_xform, current_line_pointh);
    current_line_pointh = rd_xform::multiply(clip_to_device_xform, current_line_pointh);
  }//end if
  else
  {
    // Draw to next endpoint
    float endpoint1[3];
    float endpoint2[3];

    pointh next_line_pointh(p.x, p.y, p.z, p.w);

    // Transform next endpoint to device coordinates
    next_line_pointh = rd_xform::multiply(current_xform, p);
    next_line_pointh = rd_xform::multiply(world_to_clip_xform, next_line_pointh);
    next_line_pointh = rd_xform::multiply(clip_to_device_xform, next_line_pointh);

    // Create a 3-element array from each endpoint
    // (Note: 3-element array used in line drawing algorithm)
    endpoint1[0] = current_line_pointh.x / current_line_pointh.w;
    endpoint1[1] = current_line_pointh.y / current_line_pointh.w;
    endpoint1[2] = current_line_pointh.z / current_line_pointh.w;

    endpoint2[0] = next_line_pointh.x / next_line_pointh.w;
    endpoint2[1] = next_line_pointh.y / next_line_pointh.w;
    endpoint2[2] = next_line_pointh.z / next_line_pointh.w;

    // Store "next" pointh as current for next DRAW flag
    current_line_pointh = next_line_pointh;

    // Line-drawing algorithm goes from lower X to higher X
    // so swap the two points if necessary
    if(endpoint1[0] > endpoint2[0])
    {
      for(int i = 0; i < 3; i++)
      {
	float temp = endpoint1[i];
	endpoint1[i] = endpoint2[i];
	endpoint2[i] = temp;
      }//end for
    }//end if

    // Begin drawing the line
    double deltaX = endpoint2[0] - endpoint1[0];
    double deltaY = endpoint2[1] - endpoint1[1];
    double slope = deltaY / deltaX;

    // Between 0 and 45 degrees
    if(slope >= 0 && slope <= 1)
    {
      int y = endpoint1[1];
      int p = 2 * deltaY - deltaX;

      for(int x = endpoint1[0]; x <= endpoint2[0]; x++)
      {
        rd_write_pixel(x, y, rd_direct_color.getRGB());
        if(p > 0)
        {
	  y++;
	  p += 2*deltaY - 2*deltaX;
        }//end if
        else
        {
	  p += 2*deltaY;
        }//end else
      }//end for
    }//end if
    // Greater than 45 degrees
    else if(slope > 1)
    {
      int x = endpoint1[0];
      int p = 2 * deltaX - deltaY;

      for(int y = endpoint1[1]; y <= endpoint2[1]; y++)
      {
        rd_write_pixel(x, y, rd_direct_color.getRGB());
        if(p > 0)
        {
	  x++;
	  p += 2*deltaX - 2*deltaY;
        }//end if
        else
        {
	  p += 2*deltaX;
        }//end else
      }//end for
    }//end else if
    // Between 0 and -45 degrees
    else if(slope < 0 && slope >= -1)
    {
      int y = endpoint1[1];
      int p = -2 * deltaY - deltaX;

      for(int x = endpoint1[0]; x <= endpoint2[0]; x++)
      {
        rd_write_pixel(x, y, rd_direct_color.getRGB());
        if(p > 0)
        {
	  y--;
	  p += -2*deltaY - 2*deltaX;
        }//end if
        else
        {
	  p += -2*deltaY;
        }//end else
      }//end for
    }//end else if
    else
    {
      int x = endpoint1[0];
      int p = 2 * deltaX - -1*deltaY;

      for(int y = endpoint1[1]; y >= endpoint2[1]; y--)
      {
        rd_write_pixel(x, y, rd_direct_color.getRGB());
        if( p > 0)
        {
	  x++;
	  p += 2*deltaX - -2*deltaY;
        }//end if
        else
        {
	  p += 2*deltaX;
        }//end else
      }//end for
    }//end else    
  }//end else
}//end line_pipeline
