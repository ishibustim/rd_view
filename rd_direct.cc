#include <iostream>
#include <string>
#include "color.h"
#include "rd_direct.h"
#include "rd_display.h"
#include "rd_error.h"

using std::string;
using std::clog;
using std::endl;

static color rd_direct_color(1, 1, 1);
static color rd_direct_bgColor(0, 0, 0);
static int rd_direct_frameNumber;

void find_span(int& x_start, int& x_end, int y, color& seed_color);
void fff4(int x_start, int x_end, int y, color& seed_color, color& fill_color);

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
  float endpoint1[3];
  float endpoint2[3];

  if(start[0] <= end[0])
  {
    for(int i = 0; i < 3; i++)
    {
      endpoint1[i] = start[i];
      endpoint2[i] = end[i];
    }//end for
  }//end if
  else
  {
    for(int i = 0; i < 3; i++)
    {
      endpoint1[i] = end[i];
      endpoint2[i] = start[i];
    }//end for
  }//end else

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

  return RD_OK;
}//end rd_line

int REDirect::rd_point(const float p[3])
{
  rd_write_pixel(p[0], p[1], rd_direct_color.getRGB());
  return RD_OK;
}//end rd_point

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
