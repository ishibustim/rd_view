#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include "color.h"
#include "pnm_display.h"
#include "rd_error.h"

using std::string;
using std::ofstream;
using std::ostringstream;
using std::setfill;
using std::setw;

static color** pixelGrid;
static int frameNumber;
static color bgColor(0, 0, 0);

extern string display_name;
extern int display_xSize;
extern int display_ySize;

int pnm_init_display(void)
{
  pixelGrid = new color*[display_xSize];
  for(int i = 0; i < display_xSize; i++)
  {
    pixelGrid[i] = new color[display_ySize];
  }//end for

  return RD_OK;
}//end pnm_init_display

int pnm_end_display(void)
{
  for(int i = 0; i < display_xSize; i++)
  {
    delete pixelGrid[i];
  }//end for
  delete pixelGrid;

  return RD_OK;
}//end pnm_end_display

int pnm_init_frame(int frame)
{
  frameNumber = frame;
  for(int i = 0; i < display_xSize; i++)
  {
    for(int j = 0; j < display_ySize; j++)
    {
      pixelGrid[i][j].setColor(bgColor);
    }//end for
  }//end for

  return RD_OK;
}//end pnm_init_frame

int pnm_end_frame(void)
{
  ostringstream filename;
  filename.str();
  filename << display_name <<  "-" << setfill('0') << setw(4) << frameNumber << ".ppm";

  ofstream outFile;
  outFile.open(filename.str().c_str());

  if(outFile)
  {
    outFile << "P6\n" << display_xSize << " " << display_ySize << "\n255\n";

    for(int y = 0; y < display_ySize; y++)
    {
      for(int x = 0; x < display_xSize; x++)
      {
	outFile.put(pixelGrid[x][y].red * 255.999);
	outFile.put(pixelGrid[x][y].green * 255.999);
	outFile.put(pixelGrid[x][y].blue * 255.999);
      }//end for
    }//end for
  }//end if
  outFile.close();

  return RD_OK;
}//end pnm_end_frame

int pnm_write_pixel(int x, int y, const float rgb[])
{
  float r = rgb[0];
  float g = rgb[1];
  float b = rgb[2];

  if(x >= 0 && x < display_xSize && y >= 0 && y < display_ySize)
    pixelGrid[x][y].setColor(r, g, b);

  return RD_OK;
}//end pnm_write_pixel

int pnm_read_pixel(int x, int y, float rgb[])
{
  if(x >= 0 && x < display_xSize && y >= 0 && y < display_ySize)
  {
    rgb[0] = pixelGrid[x][y].red;
    rgb[1] = pixelGrid[x][y].green;
    rgb[2] = pixelGrid[x][y].blue;
  }//end if

  return RD_OK;
}//end pnm_read_pixel

int pnm_set_background(const float rgb[])
{
  float r = rgb[0];
  float g = rgb[1];
  float b = rgb[2];

  bgColor.setColor(r, g, b);

  return RD_OK;
}//end pnm_set_background

int pnm_clear(void)
{
  for(int i = 0; i < display_xSize; i++)
  {
    for(int j = 0; j < display_ySize; j++)
    {
      pixelGrid[i][j].setColor(bgColor);
    }//end for
  }//end for

  return RD_OK;
}//end pnm_clear
