#ifndef RD_BASE_ENGINE_H
#define RD_BASE_ENGINE_H

#include <string>
using std::string;

class RenderEngine
{
public:

  /**********************   General functions  *******************************/

  virtual int rd_display(const string & name, const string & type, 
			 const string & mode);

  virtual int rd_format(int xresolution, int yresolution);

  virtual int rd_world_begin(void);
  virtual int rd_world_end(void);

  virtual int rd_frame_begin(int frame_no);
  virtual int rd_frame_end(void);

  virtual int rd_render_init(void);  /* Initialize renderer */
  virtual int rd_render_cleanup(void);

  /**********************   Camera  ******************************************/

  virtual int rd_camera_eye(const float eyepoint[3]);
  virtual int rd_camera_at(const float atpoint[3]);
  virtual int rd_camera_up(const float up[3]);
  virtual int rd_camera_fov(float fov);
  virtual int rd_clipping(float znear, float zfar);

  /**********************   Transformations **********************************/

  virtual int rd_translate(const float offset[3]);
  virtual int rd_scale(const float scale_factor[3]);
  virtual int rd_rotate_xy(float angle);
  virtual int rd_rotate_yz(float angle);
  virtual int rd_rotate_zx(float angle);

  virtual int rd_xform_push(void);
  virtual int rd_xform_pop(void);

  /**********************   Geometric Objects  *******************************/

  virtual int rd_bezier_curve(const string & vertex_type,
			      int degree, const float * vertex);

  virtual int rd_bezier_patch(const string & vertex_type,
			      int u_degree, int v_degree, 
			      const float * vertex);

  virtual int rd_catmull_clark_sds(const string & vertex_type,
				   float * coord, int nvertex,
				   int * vertex_list, int nface,
				   int * crease_list, int ncrease,
				   float *sharpness);

  virtual int rd_circle(const float center[3], float radius);

  virtual int rd_line(const float start[3], const float end[3]);

  virtual int rd_lineset(const string & vertex_type,
			 int nvertex, const float * vertex,
			 int nseg, const int * seg);

  virtual int rd_point(const float p[3]);

  virtual int rd_pointset(const string & vertex_type,
			  int nvertex, const float * vertex);
  virtual int rd_polyset(const string & vertex_type, 
			 int nvertex, const float * vertex,
			 int nface,   const int * face);

  virtual int rd_cone(float height, float radius, float thetamax);
  virtual int rd_cube(void);
  virtual int rd_cylinder(float radius, float zmin, 
			  float zmax, float thetamax);
  virtual int rd_disk(float height, float radius, float theta);

  virtual int rd_hyperboloid(const float start[3], const float end[3], 
			     float thetamax); 

  virtual int rd_paraboloid(float rmax, float zmin, 
			    float zmax, float thetamax);
  virtual int rd_sphere(float radius, float zmin, float zmax, float thetamax);
  virtual int rd_sqsphere(float radius, float north, float east, 
			  float zmin, float zmax, float thetamax); 
  virtual int rd_sqtorus(float radius1, float radius2, 
			 float north, float east, float phimin, float phimax, 
			 float thetamax);
  virtual int rd_torus(float radius1, float radius2, 
		       float phimin, float phimax, float thetamax);
  virtual int rd_tube(const float start[3], const float end[3], float radius);



  /********************  Lighting & Shading  ***************************/

  virtual int rd_background(const float color[]);
  // red, green, blue by default

  virtual int rd_color(const float color[]);

  virtual int rd_opacity(float opacity);

  virtual int rd_emission(const float color[], float intensity);

  virtual int rd_fill(const float seed_point[3]);

  virtual int rd_surface(const string & shader_type);

  virtual int rd_cone_light(const float pos[3], const float at[3], 
			    float theta_min, float theta_max,
			    const float color[], float intensity);

  virtual int rd_point_light(const float pos[3], 
			     const float color[], float intensity);

  virtual int rd_far_light  (const float dir[3], 
			     const float color[], float intensity);

  virtual int rd_ambient_light(const float color[], float intensity);


  virtual int rd_specular_color(const float color[], int exponent);

  virtual int rd_k_ambient(float Ka);
  virtual int rd_k_diffuse(float Kd);
  virtual int rd_k_emission(float Ke);
  virtual int rd_k_specular(float Ks);

  /****************************   Mapping ******************************/

  virtual int rd_map_border(const string & map_type,
			    const string & horizontal, 
			    const string & vertical);
  virtual int rd_map_bound(const string & map_type,
			   float s_min, float t_min, 
			   float s_max, float t_max);
  virtual int rd_map_load(const string & filename, 
			  const string & label);
  virtual int rd_map_sample(const string & map_type,
			    const string & intra_level, 
			    const string & inter_level);
  virtual int rd_map(const string & map_type, const string & label);


  /****************************  Options  **********************************/

  virtual int rd_option_array(const string & name, int n, const float *values);

  virtual int rd_option_bool(const string & name, bool flag);

  virtual int rd_option_list(const string & name, int n, const string values []);

  virtual int rd_option_real(const string & name, float value);

  virtual int rd_option_string(const string & name, const string & value);

  virtual int rd_custom(const string & label);

  virtual ~RenderEngine();
};


// Some useful helper functions
int get_vertex_size(const string & vertex_type);
// Returns the number of components in an attributed vertex type

#endif /* RD_BASE_ENGINE_H */
