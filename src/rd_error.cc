#include "rd_error.h"
#include "rd_parse.h"

#include <iostream>

using std::cerr;
using std::endl;

void rd_print_error(int err, const char *filename)
{
  cerr << "Error in " << filename << " on line " << rd_input_line << ": ";
  
  switch(err)
    {
    case RD_INPUT_FILE_NOT_FOUND:
      cerr << "File not found."  << endl;
      break;
    case RD_INPUT_UNKNOWN_TOKEN:
      cerr << "Unrecognized symbol" << endl;
      break;
    case RD_INPUT_ILLEGAL_TOKEN:
      cerr << "Known symbol in illegal context" << endl;
      break;
    case RD_INPUT_UNEXPECTED_EOF:
      cerr << "Unexepected end of file" << endl;
      break;
    case RD_INPUT_RUNAWAY_STRING:
      cerr << "Runaway string" << endl;
      break;
    case RD_INPUT_UNKNOWN_AXIS:
      cerr << "Unrecognized rotation axis" << endl;
      break;
    case RD_INPUT_OBJ_LIST_OVERFLOW:
      cerr << "Object list overflow" << endl;
      cerr << "Increase OBJECT_LIST_MAX_SIZE in rd_parse.C" << endl;
      break;
    case RD_INPUT_CIRCULAR_INSTANCE:
      cerr << "Recursive object definition not allowed" << endl;
      break;
    case RD_INPUT_UNKNOWN_VERTEX_TYPE:
      cerr << "Unknown vertex type combination" << endl;
      break;
    case RD_INPUT_ILLEGAL_VERTEX_INDEX:
      cerr << "Vertex index out of range" << endl;
      break;
    case RD_INPUT_UNKNOWN_OBJECT_INSTANCE:
      cerr << "Unknown object label on ObjectInstance" << endl;
      break;
    case RD_INPUT_DUPLICATE_OBJ_LABEL:
      cerr << "Duplicate object label" << endl;
      break;
    case RD_INPUT_INTERLEAVED_BLOCK:
      cerr << "Illegal block interleaving" << endl;
      break;
    case RD_INPUT_UNKNOWN_DISPLAY_TYPE:
      cerr << "Unknown display type" << endl;
      break;
    case RD_INPUT_UNKNOWN_DISPLAY_MODE:
      cerr << "Unknown display mode" << endl;
      break;
    case RD_INPUT_UNINITIALIZED_DISPLAY:
      cerr << "Display not initialized" << endl;
      break;
    case RD_INPUT_UNAVAILABLE_DISPLAY:
      cerr << "Can't open display" << endl;
      break;
    case RD_INPUT_DISPLAY_INITIALIZATION_ERROR:
      cerr << "Unspecified display initialization error" << endl;
      break;
    case RD_INPUT_UNKNOWN_SURFACE_TYPE:
      cerr << "Unknown surface material type" << endl;
      break;
    case RD_INPUT_LIGHT_LIST_OVERFLOW:
      cerr << "Light list overflow" << endl;
      break;
    case RD_INPUT_UNKNOWN_CURVE_TYPE:
      cerr << "Unknown curve type" << endl;
      break;
    case RD_INPUT_UNKNOWN_PATCH_TYPE:
      cerr << "Unknown patch type" << endl;
      break;
    case RD_INPUT_INSUFFICIENT_DISPLAY_MEMORY:
      cerr << "Insufficient memory for display buffer" << endl;
      break;
    case RD_INPUT_INSUFFICIENT_ZBUFFER_MEMORY:
      cerr << "Insufficient memory for depth buffer" << endl;
      break;
    case RD_INPUT_TRANSFORM_STACK_OVERFLOW:
      cerr << "Transform stack overflow" << endl;
      break;
    case RD_INPUT_TRANSFORM_STACK_UNDERFLOW:
      cerr << "Transform stack underflow" << endl;
      break;
    case RD_INPUT_MAP_LIST_OVERFLOW:
      cerr << "Map list overflow" << endl;
      break; 
    case RD_INPUT_MAP_FILE_NOT_FOUND:
      cerr << "Map file not found" << endl;
      break; 
    case RD_INPUT_MAP_BAD_FORMAT:
      cerr << "Map file in unrecognized format" << endl;
      break;
    case RD_INPUT_MAP_ILLEGAL_LABEL:
      cerr << "Map label already in use or reserved" << endl;
      break;
    case RD_INPUT_MAP_NOT_FOUND:
      cerr << "Map not found" << endl;
      break;
    case RD_INPUT_MAP_BAD_INTERP:
      cerr << "Illegal map interpolation type" << endl;
      break;
    case RD_INPUT_MAP_BAD_BORDER:
      cerr << "Illegal map border type" << endl;
      break;
    case RD_INPUT_UNKNOWN_ENGINE:
      cerr << "Unknown rendering engine" << endl;
      break;
    case RD_INPUT_NON_INTEGER_VALUE:
      cerr << "Integer expected"  << endl;
      break;
    case RD_INPUT_EXPECTED_VERTEX_INDEX:
      cerr << "Vertex index expected"  << endl;
      break;
    case RD_INPUT_EXPECTED_VERTEX_COORD:
      cerr << "Vertex coordinate expected"  << endl;
      break;
    case RD_INPUT_EXPECTED_REAL:
      cerr << "Floating point value expected"  << endl;
      break;
    case RD_INPUT_UNKNOWN_OPTION:
      cerr << "Unknown option" << endl;
      break;
    case RD_INPUT_BAD_BSPLINE_DEGREE:
      cerr << "Mismatch of B-spline degree, #control points, #knots" << endl;
    case RD_INPUT_BAD_OBJECT_BLOCK_PARAMETER:
      cerr << "Illegal number of parameters in object block" << endl;
    case RD_INPUT_BAD_OBJECT_INSTANCE_PARAMETER:
      cerr << "Illegal object instance parameter" << endl;
   default:
      cerr << "Unknown error code: " << err << endl;
    }
}


