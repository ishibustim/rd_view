#include "rd_parse.h"
#include "rd_display.h"
#include "rd_error.h"
#include "rd_engine.h"
#include "rd_object.h"
#include "rd_refresh.h"

#include <iostream>
#include <fstream>
#include <string>
#include <deque>
#include <cctype>
#include <cstdlib>
#include <cstring>  // memcpy()

using std::istream;
using std::string;
using std::ifstream;
using std::cin;
using std::deque;
using std::streampos;
using std::ios;

/****************************************************************************/
// Function declarations

static void EatSpace(istream& );
static int start_instance(const string & label);
static int end_instance(void);
static int get_double(istream& istr, double&);
static int get_integer(istream& istr, int&);
static int get_token(istream& istr, string& );
static int grow_float_array(float * & data, int & max_size);
static int grow_int_array(int * & data, int & max_size);
static int parse_curve(istream& istr);
static int parse_lineset(istream& istr);
static int parse_patch(istream& istr);
static int parse_pointset(istream& istr);
static int parse_polyset(istream& istr);
static int parse_subdivision(istream& istr);
static int read_bool(istream&, bool& b);
static int read_c_string(istream&, string& s);
static int store_object_header(istream& istr);

static bool outside_blocks(void);  
// True when processing is outside all blocks
static bool in_object_definition(void);
// True when inside the definition of an object (object block) but not
// as part of an object instance
static bool model_ok(void);
// True when a modeling primitive or operation is in the acceptable scope.

/****************************************************************************/
// Global variables

static ifstream rd_file;
static istream * rd_input = 0;
static bool input_pipe_flag = false;
int rd_input_line = 0;
static int err = 0;


static deque<string> token_store;  

const int STRING_TOK_MAX_SIZE = 300;

struct ObjectInstance
{
  string label;
  int n_params;
  bool visit;
  deque<ParsedObject *> object;

  ObjectInstance * prev_obj_instance;
  deque<ParsedObject *>::iterator prev_object;
  
  ObjectInstance() {visit = false; prev_obj_instance = 0;}
  ~ObjectInstance();
};

ObjectInstance::~ObjectInstance()
{
  deque<ParsedObject *>::iterator p = object.begin();

  // Free up objects pointed at by list
  while(p != object.end())
    {
      delete *p;
      ++p;
    }
}

static deque<ObjectInstance *>  ObjectInstanceCache;
static ObjectInstance * cur_obj_instance;
static deque<ParsedObject *>::iterator cur_object = 
  deque<ParsedObject *>().end();

const int VERTEX_LIST_INIT_SIZE = 50000;
static int * vertex_list;
static int vertex_list_size = 0;

enum block_type{bt_object, bt_frame, bt_world};

// Number of nested levels in certain block types
int object_block_levels = 0;
int frame_block_levels = 0;
int world_block_levels = 0;

int object_instance_levels = 0;

static deque<block_type>  parsed_block_stack;

bool display_setup = false; // Is the display set up?

/*********************************** ParsedObjectInstance *******************/
// This may eventually have to be moved and hooks made available to 
// start_instance() and end_instance().  Right now it seems somewhat 
// awkward, but there are pointer maintenance issues in other implementations.

class ParsedObjectInstanceObject : public ParsedObject
{
  string label;
public:
  ParsedObjectInstanceObject(const string & l)
    : label(l) {}
  
  virtual int apply(RenderEngine &) const;

  virtual ~ParsedObjectInstanceObject();
};

int ParsedObjectInstanceObject::apply(RenderEngine & eng) const
{
  int err;
  if((err = start_instance(label))) return err;

  // Pointers have been moved.  Apply action for new object.
  return (*cur_object)->apply(*render_engine);
}

ParsedObjectInstanceObject::~ParsedObjectInstanceObject()
{
}

/****************************************************************************/

static void	EatSpace(istream& file)
{
  // Eat white space and comments
  
  int dummychar;
  
  int	valid = 0;
  
  // eat whitespace and comments
  
  do{
    dummychar = file.get();
    if(file.eof()) return;               // Read past end of file
    switch(dummychar){
    case ' ':                            // Eat white space
    case '\t':
    case '\r':   // Needed for DOS files
      break;
    case '\n':
      rd_input_line++;
      break;
    case '#':                            // Eat comments
      while('\n' != file.get() && !file.eof());
      if(file.eof())  return;
      rd_input_line++;
      break;
    default:
      file.putback(dummychar);
      valid = 1;
      break;
    }
  }while(!valid);
}

static int read_bool(istream& istr, bool & b)
{
  int err = RD_OK;

  EatSpace(istr);

  string s;

  istr >> s;

  if(s == "on")
    b = true;
  else if(s == "off")
    b = false;
  else if(s == "true")
    b = true;
  else if(s == "false")
    b = false;
  else if(s == "yes")
    b = true;
  else if(s == "no")
    b = false;
  else if(s == "1")
    b = true;
  else if(s == "0")
    b = false;
  else err = RD_INPUT_ILLEGAL_FLAG_VALUE;

  return err;
}

static int read_c_string(istream& istr, string & s)
{
  int c;
  int count = 0;
  int tmp;
  int quote_delim_flag = 0;
  int done = 0;
  int err = 0;

  s = "";

  EatSpace(istr);
  if(istr.eof()) return RD_INPUT_UNEXPECTED_EOF;

  c = istr.get();

  if(c == '"'){
    quote_delim_flag = 1;
  }
  else
    s += c;

  while(!done)
    {
      c = istr.get(); count++;
      switch(c)
	{
	case ' ':
	case '\t':
	case '\n':
	  if(!quote_delim_flag)
	    {
	      done = 1;  // End of string
	      istr.putback(c);   // For some picky situations
	    }
	  else
	    s += c;
	  break;
	case '\"':  // "
	  if(quote_delim_flag)
	    done = 1;
	  else
	    s += c;
	  break;
	case '\\':  // Escape characters
	  c = istr.get(); count++;
	  switch(c)
	    {
	    case '\n': rd_input_line++; break;  // Concatenate over lines
	    case '\'': s += '\''; break;
	    case '\"': s += '\"'; break; // "
	    case '\\': s += '\\'; break;
	    case 'a':  s += '\a'; break;
	    case 'b':  s += '\b'; break;
	    case 'f':  s += '\f'; break;
	    case 'n':  s += '\n'; break;
	    case 'r':  s += '\r'; break;
	    case 't':  s += '\t'; break;
	    case 'v':  s += '\v'; break;
	    case 'x':  // Hex number
	      if(isdigit(c))
		tmp = c - '0';
	      else
		tmp = toupper(c) - 'A' + 10;
	      tmp <<= 4;
	      c = istr.get(); count++;
	      // Assuming again
	      if(isdigit(c))
		tmp += c - '0';
	      else
		tmp += toupper(c) - 'A' + 10;
	      break;
	    case '0':  // Octal number or \0
	    case '1':
	    case '2':
	      tmp = c - '0';
	      if(c == '0') // Check for \0
		{
		  c = istr.get(); count++;
		  if(c < '0' || c > '7')  // Not octal
		    { istr.putback(c); s += (char) 0; break;}
		}
	      else 
		c = istr.get(); count++;

	      // Assuming octal second digit
	      tmp <<= 3;  tmp += c - '0';
	      c = istr.get(); count++;
	      // Assuming octal third digit
	      tmp <<= 3;  tmp += c - '0';
	      break;
	    default:
	      s += c;
	      break;
	    }
	  break;
	default:
	  s += c;
	  break;
	}
      if(count == STRING_TOK_MAX_SIZE)
	break;
    }
  
  if(count == STRING_TOK_MAX_SIZE) err = RD_INPUT_RUNAWAY_STRING;

  if(istr.eof()) err = RD_INPUT_UNEXPECTED_EOF;

  return err;
}

static int grow_float_array(float * & data, int & max_size)
{
  // Reallocate
  int new_size = (int)(1.4 * max_size);
  float * new_list = new float[new_size];

  memcpy(new_list, data, max_size * sizeof(float));

  delete [] data;
  max_size = new_size;

  data = new_list;

  return 0;  // ???
}

static int grow_int_array(int * & data, int & max_size)
{
  // Reallocate
  int new_size = (int)(1.4 * max_size);
  int * new_list = new int[new_size];

  memcpy(new_list, data, max_size * sizeof(int));

  delete [] data;
  max_size = new_size;

  data = new_list;

  return 0;  // ???
}

static int end_instance(void)
{
  // Restore transformation state
  // Automatic transform state restoration has been removed from
  // ObjectBlocks.  The persistence of state is occasionally useful.
  // if((err = render_engine->rd_xform_pop())) return err;

  // Restore positions
  cur_obj_instance->visit = false;
  cur_object = cur_obj_instance->prev_object;
  cur_obj_instance = cur_obj_instance->prev_obj_instance;

  object_instance_levels--;
  
  if(object_instance_levels)
    ++cur_object;    // Move to next object if not back at root of instance
  
  return RD_OK;
}

static int start_instance(const string & label)
{
  // Find this instance in the list
  deque<ObjectInstance *>::iterator p = ObjectInstanceCache.begin();
  for(;p != ObjectInstanceCache.end(); ++p)
    {
      if((*p)->label == label) break;
    }
  
  if(p == ObjectInstanceCache.end()) 
    return RD_INPUT_UNKNOWN_OBJECT_INSTANCE;

  if((*p)->visit == true)
    // Currently visiting this object
    return RD_INPUT_CIRCULAR_INSTANCE;

  // Store current locations
  (*p)->prev_object = cur_object;  // May need to increment ? 
  (*p)->prev_obj_instance = cur_obj_instance;
  (*p)->visit = true;

  // Jump to the new location.
  cur_obj_instance = *p;
  cur_object = (*p)->object.begin();

  object_instance_levels++;

  // Save the current transformation state
  // The automatic save of transformation state has been removed.
  // It is occasionally useful to have state information preserved outside
  // the object block.
  // if((err = render_engine->rd_xform_push())) return err;

  return RD_OK;
}

static int parse_curve(istream& istr)
{
  string vtype; // Vertex type
  string ctype; // Curve type
  int n;        // Degree

  int vsize;
  int size;
  int err = RD_OK;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> ctype;
  EatSpace(istr);
  istr >> vtype;

  // Check curve type
  if(ctype == "\"Bezier\"")
    ;
  else
    return RD_INPUT_UNKNOWN_CURVE_TYPE;

  vsize = get_vertex_size(vtype);
  if(vsize < 0) // error code
    return vsize;

  // Different action based on curve type
  if(ctype == "\"Bezier\"")
    {
      EatSpace(istr);
      istr >> n;

      size = vsize * (n+1);
      float * coord = new float[size];
      for(int i = 0; i < size; i++)
	{
	  EatSpace(istr);
	  if(istr.eof())
	    {
	      delete [] coord;
	      return RD_INPUT_UNEXPECTED_EOF;
	    }
	  double dvalue;
	  if((err = get_double(istr, dvalue)))
	    {
	      delete [] coord;
	      return err;
	    }
	  coord[i] = dvalue;
	}
      if(!in_object_definition())
	err = render_engine->rd_bezier_curve(vtype.c_str(), n, coord);
      else
	{
	  ParsedBezierCurveObject * p = 
	    new ParsedBezierCurveObject(vtype, n, coord);
	  cur_obj_instance->object.push_back(p);
	  err = RD_OK;
	}

      delete [] coord;
    }
  else
    return RD_INPUT_UNKNOWN_CURVE_TYPE; // This shouldn't happen

  return err;
}

static int parse_lineset(istream& istr)
{
  string vtype; // Vertex type

  int vsize;
  int nvertex;
  int nseg;
  int size;

  int err;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> vtype;
  vsize = get_vertex_size(vtype);
  if(vsize < 0) // error code
    return vsize;

  EatSpace(istr);
  istr >> nvertex;
  EatSpace(istr);
  istr >> nseg;

  size = vsize * nvertex;
  float * coord = new float[size];
  for(int i = 0; i < size; i++)
    {
      EatSpace(istr);
      if(istr.eof())
	{
	  delete [] coord;
	  return RD_INPUT_UNEXPECTED_EOF;
	}
      double dvalue = 0.0;
      if((err = get_double(istr, dvalue)))
	{
	  delete [] coord;
	  return err;
	}
      coord[i] = dvalue;
    }

  int index_cnt = 0;
  int index = 0;

  for(int i = 0; i < nseg;)
    {
      EatSpace(istr);
      if(istr.eof())
	{
	  delete [] coord;
	  return RD_INPUT_UNEXPECTED_EOF;
	}
      
      if((err = get_integer(istr, index)))
	{
	  delete [] coord;
	  return err;
	}
      if(index >= nvertex)
	{
	  delete [] coord;
	  return RD_INPUT_ILLEGAL_VERTEX_INDEX;
	}
      vertex_list[index_cnt++] = index;
      if(index_cnt >= vertex_list_size)
	{
	  // Reallocate
	  int new_size = (int)(1.4 * vertex_list_size);
	  int * new_list = new int[new_size];
	  
	  memcpy(new_list, vertex_list, vertex_list_size * sizeof(int));

	  delete [] vertex_list;
	  vertex_list_size = new_size;
	  vertex_list = new_list;
	}
      if(index == -1) i++;  // New poly line
    }

  if(!in_object_definition())
    err = render_engine->rd_lineset(vtype.c_str(), nvertex, coord, nseg, 
				    vertex_list);
  else
    {
      ParsedLinesetObject * p = new ParsedLinesetObject(vtype, nvertex, coord,
							nseg, vertex_list);
      cur_obj_instance->object.push_back(p);
      err = RD_OK;
    }

  delete [] coord;

  return err;
}

static int parse_patch(istream& istr)
{
  string vtype; // Vertex type
  string ctype; // Curve type
  int u_n, v_n; // Degrees

  int vsize;
  int size;
  int err = RD_OK;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> ctype;
  EatSpace(istr);
  istr >> vtype;

  vsize = get_vertex_size(vtype);
  if(vsize < 0) // error code
    return vsize;

  // Different action based on curve type
  if(ctype == "\"Bezier\"")
    {
      EatSpace(istr);
      istr >> u_n >> v_n;

      size = vsize * (u_n+1) * (v_n+1);
      float * coord = new float[size];
      for(int i = 0; i < size; i++)
	{
	  EatSpace(istr);
	  if(istr.eof())
	    {
	      delete [] coord;
	      return RD_INPUT_UNEXPECTED_EOF;
	    }
	  double dvalue = 0.0;
	  if((err = get_double(istr, dvalue)))
	    {
	      delete [] coord;
	      return err;
	    }
	  coord[i] = dvalue;
	}
      if(!in_object_definition())
	err = render_engine->rd_bezier_patch(vtype.c_str(), u_n, v_n, coord);
      else
	{
	  ParsedBezierPatchObject * p = 
	    new ParsedBezierPatchObject(vtype, u_n, v_n, coord);
	  cur_obj_instance->object.push_back(p);
	  err = RD_OK;
	}

      delete [] coord;
    }
  else
    return RD_INPUT_UNKNOWN_PATCH_TYPE;

  return err;
}

static int parse_pointset(istream& istr)
{
  string ptype; // Point type

  int psize; // Number of values for each point
  int npoint;
  int size;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> ptype;

  psize = get_vertex_size(ptype);
  if(psize < 0) // error code
    return psize;

  EatSpace(istr);
  istr >> npoint;
  EatSpace(istr);

  size = psize * npoint;
  float * coord = new float[size];
  for(int i = 0; i < size; i++)
    {
      EatSpace(istr);
      if(istr.eof())
	{
	  delete [] coord;
	  return RD_INPUT_UNEXPECTED_EOF;
	}
      double dvalue = 0.0;
      if((err = get_double(istr, dvalue)))
	{
	  err = RD_INPUT_EXPECTED_VERTEX_COORD;

	  delete [] coord;
	  return err;
	}
      coord[i] = dvalue;
    }

  if(!in_object_definition())
    err = render_engine->rd_pointset(ptype.c_str(), npoint, coord);
  else
    {
      ParsedPointsetObject * p = new ParsedPointsetObject(ptype, 
							  npoint, coord);
      cur_obj_instance->object.push_back(p);
      err = RD_OK;
    }

  delete [] coord;

  return err;

}

static int parse_polyset(istream& istr)
{
  string vtype; // Vertex type

  int vsize;
  int nvertex;
  int nface;
  int size;

  int err;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> vtype;
  vsize = get_vertex_size(vtype);
  if(vsize < 0) // error code
    return vsize;

  EatSpace(istr);
  istr >> nvertex;
  EatSpace(istr);
  istr >> nface;

  size = vsize * nvertex;
  float * coord = new float[size];
  for(int i = 0; i < size; i++)
    {
      EatSpace(istr);
      if(istr.eof())
	{
	  delete [] coord;
	  return RD_INPUT_UNEXPECTED_EOF;
	}
      double dvalue = 0.0;
      if((err = get_double(istr, dvalue)))
	{
	  err = RD_INPUT_EXPECTED_VERTEX_COORD;

	  delete [] coord;
	  return err;
	}
      coord[i] = dvalue;
    }

  int index_cnt = 0;
  int index = 0;

  for(int i = 0; i < nface;)
    {
      EatSpace(istr);
      if(istr.eof())
	{
	  delete [] coord;
	  return RD_INPUT_UNEXPECTED_EOF;
	}
      
      if((err = get_integer(istr, index)))
	{
	  err = RD_INPUT_EXPECTED_VERTEX_INDEX;

	  delete [] coord;
	  return err;
	}
      if(index >= nvertex)
	{
	  delete [] coord;
	  return RD_INPUT_ILLEGAL_VERTEX_INDEX;
	}
      vertex_list[index_cnt++] = index;
      if(index_cnt >= vertex_list_size)
	grow_int_array(vertex_list, vertex_list_size);

      if(index == -1) i++;  // New face
    }

  if(!in_object_definition())
    err = render_engine->rd_polyset(vtype.c_str(), nvertex, coord, nface, 
				    vertex_list);
  else
    {
      ParsedPolysetObject * p = new ParsedPolysetObject(vtype, nvertex, coord,
							nface, vertex_list);
      cur_obj_instance->object.push_back(p);
      err = RD_OK;
    }

  delete [] coord;

  return err;
}

static int parse_subdivision(istream& istr)
{
  string vtype; // Vertex type
  string stype; // Surface type

  int vsize;
  int nvertex;
  int nface;
  int ncrease;
  int size;

  int err = RD_OK;

  EatSpace(istr);

  if(istr.eof())
    return RD_INPUT_UNEXPECTED_EOF;

  istr >> stype;

  EatSpace(istr);
  istr >> vtype;

  vsize = get_vertex_size(vtype);
  if(vsize < 0) // error code
    return vsize;

  // Different action based on patch type
  if(stype == "\"Catmull-Clark\"")
    {
      //Parse
      EatSpace(istr);
      istr >> nvertex;
      EatSpace(istr);
      istr >> nface;
      EatSpace(istr);
      istr >> ncrease;

      size = vsize * nvertex;
      float * coord = new float[size];
      for(int i = 0; i < size; i++)
	{
	  EatSpace(istr);
	  if(istr.eof())
	    {
	      delete [] coord;
	      return RD_INPUT_UNEXPECTED_EOF;
	    }
	  double dvalue = 0.0;
	  if((err = get_double(istr, dvalue)))
	    {
	      err = RD_INPUT_EXPECTED_VERTEX_COORD;

	      delete [] coord;
	      return err;
	    }
	  coord[i] = dvalue;
	}

      int index_cnt = 0;
      int index = 0;

      // Read faces
      for(int i = 0; i < nface;)
	{
	  EatSpace(istr);
	  if(istr.eof())
	    {
	      delete [] coord;
	      return RD_INPUT_UNEXPECTED_EOF;
	    }
	  
	  if((err = get_integer(istr, index)))
	    {
	      err = RD_INPUT_EXPECTED_VERTEX_INDEX;

	      delete [] coord;
	      return err;
	    }
	  if(index >= nvertex)
	    {
	      delete [] coord;
	      return RD_INPUT_ILLEGAL_VERTEX_INDEX;
	    }
	  vertex_list[index_cnt++] = index;
	  if(index_cnt >= vertex_list_size)
	    grow_int_array(vertex_list, vertex_list_size);

	  if(index == -1) i++; // New face
	}

      // Creases
      int crease_list_size = 10;
      int * crease_list = new int[crease_list_size];
      int crease_index = 0;

      int sharp_list_size = 10;
      float * sharp_list = new float[sharp_list_size];
      int sharp_index = 0;

      for(int i = 0; i < ncrease; i++)
	{
	  index_cnt = 0;  // The number of edges on the crease
	  // Read crease edges
	  while(1)
	    {
	      EatSpace(istr);
	      if(istr.eof())
		{
		  delete [] coord; delete [] crease_list; delete sharp_list;
		  return RD_INPUT_UNEXPECTED_EOF;
		}
	      
	      if((err = get_integer(istr, index)))
		{
		  delete [] coord; delete [] crease_list; delete sharp_list;
		  return RD_INPUT_EXPECTED_VERTEX_INDEX;
		}
	      if(index >= nvertex)
		{
		  delete [] coord; delete [] crease_list; delete sharp_list;
		  return RD_INPUT_ILLEGAL_VERTEX_INDEX;
		}

	      crease_list[crease_index++] = index;
	      if(crease_index >= crease_list_size)
		grow_int_array(crease_list, crease_list_size);

	      if(index == -1) // Done with this crease
		break;

	      index_cnt++;

	    }

	  // Read sharpness values

	  for(int j = 0; j < index_cnt - 1; j++)
	    {
	      EatSpace(istr);
	      if(istr.eof())
		{
		  delete [] coord; delete [] crease_list; delete sharp_list;
		  return RD_INPUT_UNEXPECTED_EOF;
		}
	      double dvalue;
	      if((err = get_double(istr, dvalue)))
		{
		  delete [] coord; delete [] crease_list; delete sharp_list;
		  return RD_INPUT_EXPECTED_REAL;
		}
	      sharp_list[sharp_index++] = dvalue;
	      if(sharp_index >= sharp_list_size)
		grow_float_array(sharp_list, sharp_list_size);

	    }
	}

      // Store
      ParsedCatmullClarkSDSObject * p = 
	new ParsedCatmullClarkSDSObject(vtype, 
					coord, nvertex,
					vertex_list, nface,
					crease_list, ncrease,
					sharp_list);

      if(!in_object_definition())
	{
	  err = p->apply(*render_engine);
	  delete p;
	}
      else
	{
	  cur_obj_instance->object.push_back(p);
	  err = RD_OK;
	}
      // Cleanup temp arrays here
      delete [] coord; delete [] crease_list; delete sharp_list;
    }
  else
    return RD_INPUT_UNKNOWN_PATCH_TYPE;

  return err;
}

static int store_object_header(istream& istr)
{
  string label;
  string parameters;

  // Parse and store number of parameters
  if((err = read_c_string(istr, parameters))) return err;
  
  char * param_end;
  const char * param_start = parameters.c_str();

  long int n_parameters = strtol(param_start, &param_end, 10);

  if(*param_end != '\0' && param_start != param_end)
    // Converted in part, premature end
    return RD_INPUT_BAD_OBJECT_BLOCK_PARAMETER;

  if(*param_end == '\0')
    {
      // Really found a parameter count
      // Get label
      if((err = read_c_string(istr, label))) return err;
    }
  else
    {
      // No parameter count -- really found label
      label = parameters;
    }

  // Parse and store label
  // See if this object already exists
  deque<ObjectInstance *>::iterator p = ObjectInstanceCache.begin();

  while(p != ObjectInstanceCache.end())
    {
      if((*p)->label == label) return RD_INPUT_DUPLICATE_OBJ_LABEL;
      ++p;
    }

  // New object instance to store
  ObjectInstance  * oi = new ObjectInstance;
  oi->label = label;
  oi->n_params = n_parameters;
  ObjectInstanceCache.push_back(oi);
  cur_obj_instance = oi;

  return RD_OK;
}

inline static bool outside_blocks(void)
{
  return 
    (object_block_levels == 0) && 
    (frame_block_levels == 0)  &&
    (world_block_levels == 0);
}

inline static bool in_object_definition(void)
{
  return (object_block_levels != 0) && (world_block_levels == 0);
}

inline static bool model_ok(void)
{
  return (world_block_levels != 0 || object_block_levels != 0);
}

int rd_parse_init(char * filename)
{
  int err = RD_OK;

  if(filename[0] == '-' && filename[1] == '\0')
    {
      // Piped input
      input_pipe_flag = true;
      rd_input = &cin;
    }
  else
    {
      input_pipe_flag = false;
      rd_file.open(filename, ios::in | ios::binary);

      if(!rd_file)
	return RD_INPUT_FILE_NOT_FOUND;

      rd_input = &rd_file;
    }

  // State initialization
  object_block_levels = 0;
  frame_block_levels  = 0;
  world_block_levels  = 0;

  rd_input_line = 1;

  parsed_block_stack.clear();
  token_store.clear();

  display_setup = false;

  // Array initialization
  vertex_list = new int[VERTEX_LIST_INIT_SIZE];
  vertex_list_size = VERTEX_LIST_INIT_SIZE;

  return err;
}

int rd_parse_unit(void)
{
  // Input variables for parameters
  int i1, i2;
  double f1, f2, f3, f4, f5, f6, f7;
  float p1[3], p2[3], p3[3];
  string s1, s2, s3;
  bool b1 = true;

  ParsedObject *obj;

  int err = RD_OK;

  while(1)
    {
      // If in an object instance then tokens are already parsed
      if(world_block_levels && object_instance_levels)  
	// Actually rendering parsed tokens
	{
	  if(cur_object == cur_obj_instance->object.end())
	    {
	      end_instance();  // Moves back to calling object
	      continue;
	    }

	  if((err = (*cur_object)->apply(*render_engine))) return err;
	  // This actually doesn't work if the current object is a
	  // parsed object instance.  This type of object actually pushes
	  // the current object instance on hold and starts another.
	  // This implies knowledge of hooks in the parser.  Yuck.

	  
	  ++cur_object;

	  if(refresh_query(refresh_object)) break;
	  
	  continue;  
	  // This needs to be examined from the point of view
	  // of refreshing the screen on a per-object basis.  Maybe change
	  // to break;
	}

      EatSpace(*rd_input);
      // Check for end of file
      if((*rd_input).eof()){
	if(!outside_blocks())
	  err = RD_INPUT_UNEXPECTED_EOF;
	else
	  err = RD_INPUT_EXPECTED_EOF;

	break;
      }

      string token;

      get_token(*rd_input, token);

      if(token == "AmbientLight")
	{
	  if(!world_block_levels || object_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}

	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f3;
	  ParsedAmbientLightObject a(p2, f3);
	  if((err = a.apply(*render_engine))) return err;
	}
      else if(token == "Background")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  if((err = render_engine->rd_background(p2))) break;
	  // Keep going
	}
      else if(token == "CameraAt")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if((err = render_engine->rd_camera_at(p1))) break;
	  // Keep going
	}
      else if(token == "CameraEye")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if((err = render_engine->rd_camera_eye(p1))) break;
	  // Keep going
	}
      else if(token == "CameraFOV")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if((err = render_engine->rd_camera_fov(f1))) break;
	  // Keep going
	}
      else if(token == "CameraUp")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if((err = render_engine->rd_camera_up(p1))) break;
	  // Keep going
	}
      else if(token == "Circle")
	{
	  if(!world_block_levels){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> f1;
	  if((err = render_engine->rd_circle(p1, f1))) break;
	  // Return control?
	  if(refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Clipping")
	{
	  if(model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1 >> f2;
	  if((err = render_engine->rd_clipping(f1, f2))) break;
	  // Keep going
	}
      else if(token == "Color")
	{
	  // This one's legal anywhere
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if(in_object_definition())
	    {
	      obj = new ParsedColorObject(p1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_color(p1))) break;
	  }
	  // Keep going
	}
      else if(token == "Cone")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3;

	  if(in_object_definition())
	    {
	      obj = new ParsedConeObject(f1, f2, f3);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_cone(f1, f2, f3))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "ConeLight")
	{
	  if(!world_block_levels || object_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f1 >> f2;
	  *rd_input >> p3[0] >> p3[1] >> p3[2];
	  *rd_input >> f3;

	  if((err = render_engine->rd_cone_light(p1, p2, 
						 f1, f2, p3, f3))) return err;
	}
      else if(token == "Cube")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(in_object_definition())
	    {
	      // Store cube
	      obj = new ParsedCubeObject;
	      cur_obj_instance->object.push_back(obj);
	    } 
	  else {
	    if((err = render_engine->rd_cube())) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Curve")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_curve(*rd_input))) break;
	  if(!in_object_definition() &&
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Custom")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if(in_object_definition())
	    ; // Ignore
	  else {
	    if((err = render_engine->rd_custom(s1))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Cylinder")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3 >> f4;

	  if(in_object_definition()) 
	    {
	      obj = new ParsedCylinderObject(f1, f2, f3, f4);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_cylinder(f1, f2, f3, f4))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Disk")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3;

	  if(in_object_definition()) 
	    {
	      obj = new ParsedDiskObject(f1, f2, f3);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_disk(f1, f2, f3))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Display")
	{
	  if(!outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;
	  if((err = read_c_string(*rd_input, s3))) break;
	  // Internal display setup
	  if((err = rd_set_display(s1, s2, s3))) break;
	  // Call user hook
	  if((err = render_engine->rd_display(s1, s2, s3))) break;
	  // Keep going
	}
      else if(token == "Emit")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> f3;

	  if(in_object_definition())
	    {
	      obj = new ParsedEmissionObject(p1, f3);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_emission(p1, f3))) break;
	  }
	  // Keep going
	}
      else if(token == "Engine")
	{
	  if(!outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = find_engine(s1))) break;
	  // Keep going
	}
      else if(token == "FarLight")
	{
	  if(!world_block_levels || object_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f3;

	  if((err = render_engine->rd_far_light(p1, p2, f3))) return err;
	}
      else if(token == "Fill")
	{
	  if(!world_block_levels){err = RD_INPUT_ILLEGAL_TOKEN; break;}

	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if((err = render_engine->rd_fill(p1))) break;
	  // Return control?
	  if(refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Format")
	{
	  if(!outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> i1 >> i2;
	  // Internal setup
	  if((err = rd_set_format(i1, i2))) break;
	  // Call user hook
	  if((err = render_engine->rd_format(i1, i2))) break;
	  // Keep going
	}
      else if(token == "FrameBegin")
	{
	  if(!outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> i1;
	  if((err = render_engine->rd_frame_begin(i1))) break;
	  parsed_block_stack.push_back(bt_frame);
	  frame_block_levels++;
	  // Keep going
	}
      else if(token == "FrameEnd")
	{
	  if(object_block_levels || world_block_levels || !frame_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(parsed_block_stack.back() != bt_frame)
	    {err = RD_INPUT_INTERLEAVED_BLOCK; break;}
	  parsed_block_stack.pop_back();
	  frame_block_levels--;
	  if((err = render_engine->rd_frame_end())) break;
	  // Stop here regardless
	  break;
	}
      else if(token == "Hyperboloid")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f1;

	  if(in_object_definition()) 
	    {
	      obj = new ParsedHyperboloidObject(p1, p2, f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_hyperboloid(p1, p2, f1))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Ka")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if(in_object_definition())
	    {
	      obj = new ParsedKaObject(f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_k_ambient(f1))) break;
	  }
	  // Keep going
	}
      else if(token == "Kd")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if(in_object_definition())
	    {
	      obj = new ParsedKdObject(f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_k_diffuse(f1))) break;
	  }
	  // Keep going
	}
      else if(token == "Ke")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if(in_object_definition())
	    {
	      obj = new ParsedKeObject(f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_k_emission(f1))) break;
	  }
	  // Keep going
	}
      else if(token == "Ks")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if(in_object_definition())
	    {
	      obj = new ParsedKsObject(f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_k_specular(f1))) break;
	  }
	  // Keep going
	}
      else if(token == "Line")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  if(in_object_definition()) 
	    {
	      obj = new ParsedLineObject(p1, p2);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_line(p1, p2))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "LineSet")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_lineset(*rd_input))) break;
	  if(!in_object_definition() && 
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Map")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;
	  if(in_object_definition())
	    {
	      obj = new ParsedMapObject(s1, s2);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_map(s1.c_str(), s2.c_str()))) break;
	  }
	  // Keep going
	}
      else if(token == "MapBorder")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;
	  if((err = read_c_string(*rd_input, s3))) break;

	  if(in_object_definition())
	    {
	      obj = new ParsedMapBorderObject(s1, s2, s3);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_map_border(s1.c_str(), 
						       s2.c_str(),
						       s3.c_str()))) break;
	  }
	  // Keep going
	}
      else if(token == "MapBound")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  *rd_input >> f1 >> f2 >> f3 >> f4;

	  if(in_object_definition())
	    {
	      obj = new ParsedMapBoundObject(s1, f1, f2, f3, f4);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_map_bound(s1, f1, f2, f3, f4))) break;
	  }
	  // Keep going
	}
      else if(token == "MapLoad")
	{
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;

	  if(in_object_definition())
	    {
	      obj = new ParsedMapLoadObject(s1, s2);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_map_load(s1.c_str(), 
						 s2.c_str()))) break;
	  }
	  // Keep going
	}
      else if(token == "MapSample")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;
	  if((err = read_c_string(*rd_input, s3))) break;

	  if(in_object_definition())
	    {
	      obj = new ParsedMapSampleObject(s1, s2, s3);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_map_sample(s1.c_str(), 
						   s2.c_str(),
						   s3.c_str()))) break;
	  }
	  // Keep going
	}
      else if(token == "ObjectBegin")
	{
	  if(!outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = store_object_header(*rd_input))) break;
	  parsed_block_stack.push_back(bt_object);
	  object_block_levels++;
	  // Keep going
	}
      else if(token == "ObjectEnd")
	{
	  if(!object_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(parsed_block_stack.back() != bt_object)
	    {err = RD_INPUT_INTERLEAVED_BLOCK; break;}
	  parsed_block_stack.pop_back();
	  object_block_levels--;

	  // Keep going
	}
      else if(token == "ObjectInstance")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if(world_block_levels){ // Actual instantiation
	    if((err = start_instance(s1))) break;
	  }
	  else
	    {
	      // Instance calling instance
	      obj = new ParsedObjectInstanceObject(s1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  // Keep going
	}
      else if(token == "Opacity")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1;
	  if(in_object_definition())
	    {
	      obj = new ParsedOpacityObject(f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_opacity(f1))) break;
	  }
	  // Keep going
	}
      else if(token == "OptionArray")
	{
	  if((err = read_c_string(*rd_input, s1))) break;
	  *rd_input >> i1;
	  float * val = new float[i1];
	  for(int i = 0; i < i1; i++)
	    *rd_input >> val[i];

	  if(in_object_definition())
	    {
	      obj = new ParsedOptionArrayObject(s1, i1, val);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else
	    err = render_engine->rd_option_array(s1.c_str(), i1, val);

	  delete [] val;
	}
      else if(token == "OptionBool")
	{
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_bool(*rd_input, b1))) break;
	  
	  if(in_object_definition())
	    {
	      obj = new ParsedOptionBoolObject(s1, b1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else
	    err = render_engine->rd_option_bool(s1, b1);
	}
      else if(token == "OptionList")
	{
	}
      else if(token == "OptionReal")
	{
	  if((err = read_c_string(*rd_input, s1))) break;
	  *rd_input >> f1;
	  
	  if(in_object_definition())
	    {
	      obj = new ParsedOptionRealObject(s1, f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else
	    err = render_engine->rd_option_real(s1, f1);
	}
      else if(token == "OptionString")
	{
	  if((err = read_c_string(*rd_input, s1))) break;
	  if((err = read_c_string(*rd_input, s2))) break;
	  
	  if(in_object_definition())
	    {
	      obj = new ParsedOptionStringObject(s1, s2);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else
	    err = render_engine->rd_option_string(s1, s2);
	}
      else if(token == "Paraboloid")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3 >> f4;

	  if(in_object_definition()) 
	    {
	      obj = new ParsedParaboloidObject(f1, f2, f3, f4);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_paraboloid(f1, f2, f3, f4))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Patch")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_patch(*rd_input))) break;
	  if(!in_object_definition() &&
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Point")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if(in_object_definition())
	    {
	      ParsedPointObject * p = new ParsedPointObject(p1);
	      cur_obj_instance->object.push_back(p);
	    }
	  else {
	    if((err = render_engine->rd_point(p1))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "PointLight")
	{
	  if(!world_block_levels || object_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f3;

	  if((err = render_engine->rd_point_light(p1, p2, f3))) return err;
	}
      else if(token == "PointSet")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_pointset(*rd_input))) break;
	  if(!in_object_definition() &&
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "PolySet")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_polyset(*rd_input))) break;
	  if(!in_object_definition() && 
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Rotate")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input); *rd_input >> s1;
	  EatSpace(*rd_input); *rd_input >> f1;
	  
	  if(in_object_definition())
	    {
	      if(s1 == "\"X\"" || s1 == "\"x\"")
	        obj = new ParsedRotateYZObject(f1);
	      else if(s1 == "\"Y\"" || s1 == "\"y\"")
		obj = new ParsedRotateZXObject(f1);
	      else if(s1 == "\"Z\"" || s1 == "\"z\"")
		obj = new ParsedRotateXYObject(f1);
	      else{ err = RD_INPUT_UNKNOWN_AXIS; break;}

	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if(     s1 == "\"X\"" || s1 == "\"x\"") 
	      err = render_engine->rd_rotate_yz(f1);
	    else if(s1 == "\"Y\"" || s1 == "\"y\"") 
	      err = render_engine->rd_rotate_zx(f1);
	    else if(s1 == "\"Z\"" || s1 == "\"z\"") 
	      err = render_engine->rd_rotate_xy(f1);
	    else err = RD_INPUT_UNKNOWN_AXIS;

	    if(err) break;
	  }
	  // Keep going
	}
      else if(token == "Scale")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if(in_object_definition())
	    {
	      // Store scale parameters
	      ParsedScaleObject *s = new ParsedScaleObject(p1);
	      cur_obj_instance->object.push_back(s);
	    }
	  else {
	    if((err = render_engine->rd_scale(p1))) break;
	  }
	  // Keep going
	}
      else if(token == "Specular")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  EatSpace(*rd_input);
	  *rd_input >> i1;
	  
	  if(in_object_definition())
	    {
	      obj = new ParsedSpecularObject(p1, i1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_specular_color(p1, i1))) break;
	  }
	  // Keep going
	}
      else if(token == "Sphere")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3 >> f4;
	  if(in_object_definition())
	    {
	      obj = new ParsedSphereObject(f1, f2, f3, f4);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_sphere(f1, f2, f3, f4))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "SqSphere")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3 >> f4 >> f5 >> f6;
	  if(in_object_definition())
	    {
	      obj = new ParsedSqSphereObject(f1, f2, f3, f4, f5, f6);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_sqsphere(f1, f2, f3, f4, f5, f6))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "SqTorus")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  EatSpace(*rd_input);
	  *rd_input >> f1 >> f2 >> f3 >> f4 >> f5 >> f6 >> f7;
	  if(in_object_definition())
	    {
	      obj = new ParsedSqTorusObject(f1, f2, f3, f4, f5, f6, f7);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_sqtorus(f1, f2, f3, f4, 
						f5, f6, f7))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "Subdivision")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = parse_subdivision(*rd_input))) break;
	  if(!in_object_definition() &&
	     refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Surface")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if((err = read_c_string(*rd_input, s1))) break;
	  if(in_object_definition())
	    {
	      obj = new ParsedSurfaceObject(s1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_surface(s1.c_str()))) break;
	  }
	  // Keep going
	}
      else if(token == "Torus")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> f1 >> f2 >> f3 >> f4 >> f5;
	  if(in_object_definition())
	    {
	      obj = new ParsedTorusObject(f1, f2, f3, f4, f5);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_torus(f1, f2, f3, f4, f5))) break;
	  }
	  // Return control?
	  if(refresh_query(refresh_object)) break;
	  // Keep going
	}
      else if(token == "Translate")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  if(in_object_definition()) 
	    {
	      ParsedTranslateObject *t = new ParsedTranslateObject(p1);
	      cur_obj_instance->object.push_back(t);
	    }
	  else {
	    if((err = render_engine->rd_translate(p1)))break;
	  }
	  // Keep going
	}
      else if(token == "Tube")
	{
	  if(!model_ok()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  *rd_input >> p1[0] >> p1[1] >> p1[2];
	  *rd_input >> p2[0] >> p2[1] >> p2[2];
	  *rd_input >> f1;
	  if(in_object_definition()) 
	    {
	      obj = new ParsedTubeObject(p1, p2, f1);
	      cur_obj_instance->object.push_back(obj);
	    }
	  else {
	    if((err = render_engine->rd_tube(p1, p2, f1))) break;
	    // Return control?
	    if(refresh_query(refresh_object)) break;
	  }
	  // Keep going
	}
      else if(token == "WorldBegin")
	{
	  if(object_block_levels || world_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(!display_setup){  // Set up the display if it hasn't been done
	    display_setup = true;
	    token_store.push_back("WorldBegin");
	    err = RD_INPUT_UNINITIALIZED_DISPLAY;
	    break; // Some displays must be initialized outside parser.
	  }
	  if((err = render_engine->rd_world_begin())) break;
	  parsed_block_stack.push_back(bt_world);
	  world_block_levels++;
	  // Keep going
	}
      else if(token == "WorldEnd")
	{
	  if(object_block_levels || !world_block_levels)
	    {err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(parsed_block_stack.back() != bt_world)
	    {err = RD_INPUT_INTERLEAVED_BLOCK; break;}
	  parsed_block_stack.pop_back();
	  world_block_levels--;
	  if((err = render_engine->rd_world_end())) break;
	  // Stop here regardless
	  break;
	}
      else if(token == "XformPop")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(in_object_definition()) 
	    {
	      ParsedXformPopObject *x = new ParsedXformPopObject();
	      cur_obj_instance->object.push_back(x);
	    }
	  else {
	    if((err = render_engine->rd_xform_pop())) break;
	  }
	  // Keep going
	}
      else if(token == "XformPush")
	{
	  if(outside_blocks()){err = RD_INPUT_ILLEGAL_TOKEN; break;}
	  if(in_object_definition()) 
	    {
	      ParsedXformPushObject *x = new ParsedXformPushObject();
	      cur_obj_instance->object.push_back(x);
	    }
	  else {
	    if((err = render_engine->rd_xform_push())) break;
	  }
	  // Keep going
	}
      else
	{
	  err = RD_INPUT_UNKNOWN_TOKEN;
	  break;
	}
    }

  return err;
}

int rd_parse_end(void)
{
  if(input_pipe_flag)
    rd_file.close();

  // Release vertex memory
  delete [] vertex_list;
  vertex_list_size = 0;

  // Cleanup Object Instance Cache
  deque<ObjectInstance *>::iterator p = ObjectInstanceCache.begin();
  for(; p != ObjectInstanceCache.end(); ++p)
    {
      delete *p;
    }

  return RD_OK;
}

int get_token(istream & istr, string& token)
{
  if(token_store.empty())
    {
      if(istr.eof())
	return RD_INPUT_UNEXPECTED_EOF;
      istr >> token;
    }
  else
    {
      token = token_store.back();
      token_store.pop_back();
    }
  return RD_OK;
}

int get_double(istream & istr, double & d)
{
  string token;
  int err = RD_OK;
  char * end;
  
  if((err = get_token(istr, token)))
    return err;

  d = strtod(token.c_str(), &end);

  if(token.c_str() == end)
    // No conversion -- most likely due to keyword.
    return RD_INPUT_ILLEGAL_TOKEN;
  
  return RD_OK;
}

int get_integer(istream & istr, int & i)
{
  
  string token;
  int err = RD_OK;
  char * end;
  
  if((err = get_token(istr, token)))
    return err;

  i = strtol(token.c_str(), &end, 0);

  if(token.c_str() == end)
    // No conversion -- most likely due to keyword.
    return RD_INPUT_ILLEGAL_TOKEN;
  
  if(*end == '.')
    return RD_INPUT_NON_INTEGER_VALUE;
  return RD_OK;
}

