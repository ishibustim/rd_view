#include "rd_enginebase.h"
#include "rd_error.h"

#include <string>
#include <iostream>

using std::string;
using std::cerr;
using std::endl;

void common_error(const char * com)
{
  // Save some space in the text segment
  cerr << com << " command not implemented. Ignoring." << endl;
}

void option_error(const char *com, const char * opt)
{
  cerr << "Option \"" << opt << "\" not implemented for ";
  cerr << com << " command. Ignoring." << endl;
}

int RenderEngine::rd_display(const string & name, const string & type, 
			 const string & mode)
{
  common_error("Display");
  return RD_OK;
}


int RenderEngine::rd_format(int xresolution, int yresolution)
{
  common_error("Format");
   return RD_OK;
}

int RenderEngine::rd_frame_begin(int frame_no)
{
  common_error("FrameBegin");
  return RD_OK;
}

int RenderEngine::rd_frame_end(void)
{
  common_error("FrameEnd");
  return RD_OK;
}


int RenderEngine::rd_render_init(void)
{
  return RD_OK;
}
  /* Initialize renderer */
int RenderEngine::rd_render_cleanup(void)
{
  return RD_OK;
}


int RenderEngine::rd_point(const float p[3])
{
  common_error("Point");
  return RD_OK;
}

int RenderEngine::rd_line(const float start[3], const float end[3])
{
  common_error("Line");
  return RD_OK;
}

int RenderEngine::rd_translate(const float offset[3])
{
  common_error("Translate");
  return RD_OK;
}

int RenderEngine::rd_scale(const float scale_factor[3])
{
  common_error("Scale");
  return RD_OK;
}

int RenderEngine::rd_rotate_xy(float angle)
{
  common_error("Rotate");
  return RD_OK;
}

int RenderEngine::rd_rotate_yz(float angle)
{
  common_error("Rotate");
  return RD_OK;
}

int RenderEngine::rd_rotate_zx(float angle)
{
  common_error("Rotate");
  return RD_OK;
}

int RenderEngine::rd_camera_eye(const float eyepoint[3])
{
  common_error("CameraEye");
  return RD_OK;
}

int RenderEngine::rd_camera_at(const float atpoint[3])
{
  common_error("CameraAt");
  return RD_OK;
}

int RenderEngine::rd_camera_up(const float up[3])
{
  common_error("CameraUp");
  return RD_OK;
}

int RenderEngine::rd_camera_fov(float fov)
{
  common_error("CameraFOV");
  return RD_OK;
}

int RenderEngine::rd_clipping(float znear, float zfar)
{
  common_error("Clipping");
  return RD_OK;
}

int RenderEngine::rd_world_begin(void)
{
  common_error("WorldBegin");
  return RD_OK;
}

int RenderEngine::rd_world_end(void)
{
  common_error("WorldEnd");
  return RD_OK;
}


int RenderEngine::rd_bezier_curve(const string & vertex_type,
			      int degree, const float * vertex)
{
  option_error("Curve", "Bezier");
  return RD_OK;
}

int RenderEngine::rd_bezier_patch(const string & vertex_type,
			      int u_degree, int v_degree, const float * vertex)
{
  option_error("Patch", "Bezier");
  return RD_OK;
}

int RenderEngine::rd_catmull_clark_sds(const string & vertex_type,
				       float * coord, int nvertex,
				       int * vertex_list, int nface,
				       int * crease_list, int ncrease,
				       float *sharpness)
{
  option_error("Subdivision", "Catmull-Clark");
  return RD_OK;
}

int RenderEngine::rd_lineset(const string & vertex_type, 
			 int nvertex, const float * vertex,
			 int nseg,   const int * seg)
{
  common_error("LineSet");
  return RD_OK;
}

int RenderEngine::rd_pointset(const string & vertex_type,
			  int nvertex, const float * vertex)
{
  common_error("PointSet");
  return RD_OK;
}

int RenderEngine::rd_polyset(const string & vertex_type, 
			 int nvertex, const float * vertex,
			 int nface,   const int * face)
{
  common_error("PolySet");
  return RD_OK;
}


int RenderEngine::rd_cube(void)
{
  common_error("Cube");
  return RD_OK;
}

int RenderEngine::rd_sphere(float radius, float zmin, float zmax, float thetamax)
{
  common_error("Sphere");
  return RD_OK;
}

int RenderEngine::rd_sqsphere(float radius, float n, float e,
			      float zmin, float zmax, float thetamax)
{
  common_error("SqSphere");
  return RD_OK;
}

int RenderEngine::rd_sqtorus(float radius1, float radius2, float n, float e,
			     float phimin, float phimax, float thetamax)
{
  common_error("SqTorus");
  return RD_OK;
}

int RenderEngine::rd_cone(float height, float radius, float thetamax)
{
  common_error("Cone");
  return RD_OK;
}

int RenderEngine::rd_disk(float height, float radius, float theta)
{
  common_error("Disk");
  return RD_OK;
}

int RenderEngine::rd_cylinder(float radius, float zmin, 
			  float zmax, float thetamax)
{
  common_error("Cylinder");
  return RD_OK;
}

int RenderEngine::rd_tube(const float start[3], const float end[3], float radius)
{
  common_error("Tube");
  return RD_OK;
}

int RenderEngine::rd_hyperboloid(const float start[3], const float end[3], 
				 float thetamax)
{
  common_error("Hyperboloid");
  return RD_OK;
}

int RenderEngine::rd_paraboloid(float rmax, float zmin, 
			    float zmax, float thetamax)
{
  common_error("Paraboloid");
  return RD_OK;
}

int RenderEngine::rd_torus(float radius1, float radius2, 
		       float phimin, float phimax, float thetamax)
{
  common_error("Torus");
  return RD_OK;
}


int RenderEngine::rd_background(const float color[])
{
  common_error("Background");
  return RD_OK;
}

int RenderEngine::rd_color(const float color[])
{
  common_error("Color");
  return RD_OK;
}

int RenderEngine::rd_emission(const float color[], float intensity)
{
  common_error("Emit");
  return RD_OK;
}

int RenderEngine::rd_opacity(float opacity)
{
  common_error("Opacity");
  return RD_OK;
}

int RenderEngine::rd_circle(const float center[3], float radius)
{
  common_error("Circle");
  return RD_OK;
}


int RenderEngine::rd_fill(const float seed_point[3])
{
  common_error("Fill");
  return RD_OK;
}


int RenderEngine::rd_xform_push(void)
{
  common_error("XformPush");
  return RD_OK;
}

int RenderEngine::rd_xform_pop(void)
{
  common_error("XformPop");
  return RD_OK;
}

int RenderEngine::rd_option_array(const string & name, int n, 
				  const float *values)
{
  common_error("OptionArray");
  return RD_OK;
}

int RenderEngine::rd_option_bool(const string & name, bool flag)
{
  common_error("OptionBool");
  return RD_OK;
}

int RenderEngine::rd_option_list(const string & name, int n,
				 const string values [])
{
  common_error("OptionList");
  return RD_OK;
}

int RenderEngine::rd_option_real(const string & name, float value)
{
  common_error("OptionReal");
  return RD_OK;
}

int RenderEngine::rd_option_string(const string & name, const string & value)
{
  common_error("OptionString");
  return RD_OK;
}

int RenderEngine::rd_surface(const string & shader_type)
{
  common_error("Surface");
  return RD_OK;
}

int RenderEngine::rd_cone_light(const float pos[3], const float at[3],
				float theta_min, float theta_max,
				const float color[], float intensity)
{
  common_error("ConeLight");
  return RD_OK;
}

int RenderEngine::rd_point_light(const float pos[3], const float color[], 
				 float intensity)
{
  common_error("PointLight");
  return RD_OK;
}

int RenderEngine::rd_far_light  (const float dir[3], const float color[3], 
				 float intensity)
{
  common_error("FarLight");
  return RD_OK;
}

int RenderEngine::rd_ambient_light(const float color[], float intensity)
{
  common_error("AmbientLight");
  return RD_OK;
}


int RenderEngine::rd_specular_color(const float color[], int exponent)
{
  common_error("Specular");
  return RD_OK;
}


int RenderEngine::rd_k_ambient(float Ka)
{
  common_error("Ka");
  return RD_OK;
}

int RenderEngine::rd_k_diffuse(float Kd)
{
  common_error("Kd");
  return RD_OK;
}

int RenderEngine::rd_k_emission(float Ke)
{
  common_error("Ke");
  return RD_OK;
}

int RenderEngine::rd_k_specular(float Ks)
{
  common_error("Ks");
  return RD_OK;
}


int RenderEngine::rd_map_border(const string & map_type,
				const string & horizontal, 
				const string & vertical)
{
  common_error("MapBorder");
  return RD_OK;
}

int RenderEngine::rd_map_bound(const string & map_type,
			       float s_min, float t_min, 
			       float s_max, float t_max)
{
  common_error("MapBound");
  return RD_OK;
}

int RenderEngine::rd_map_load(const string & filename, 
			      const string & label)
{
  common_error("MapLoad");
  return RD_OK;
}

int RenderEngine::rd_map_sample(const string & map_type,
				const string & inter_level, 
				const string & intra_level)
{
  common_error("MapSample");
  return RD_OK;
}

int RenderEngine::rd_map(const string & map_type, const string & name)
{
  common_error("Map");
  return RD_OK;
}


int RenderEngine::rd_custom(const string & label)
{
  common_error("Custom");
  return RD_OK;
}

RenderEngine::~RenderEngine()
{
}

int get_vertex_size(const string & vertex_type)
{
  int err = RD_OK;
  const char * attribute = &vertex_type[0];
  int i = vertex_type.length();
  int vertex_size = 0;
  
  for(; i; attribute++, i--)
    {
      switch(*attribute)
	{
	case '\"': // Ignore \" 
	  break;
	case 'P':
	  vertex_size += 3;
	  break;
	case 'D': case 'N':
	  vertex_size += 3;
	  break;
	case 'C':
	  vertex_size += 3;
	  break;
	case 'w':
	  vertex_size += 1;
	  break;
	case 'T':
	  vertex_size += 2;
	  break;
	default:
	  err = RD_INPUT_UNKNOWN_VERTEX_TYPE;
	  break;
	}
    }
  if(err != RD_OK)
    return err;

  return vertex_size;

}
