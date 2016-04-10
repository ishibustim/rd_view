#include "rd_object.h"
#include "rd_enginebase.h"
#include "rd_parse.h" // hook needed for ParsedObjectInstanceObject::apply()
#include <cstring>  // memcpy()

ParsedObject::~ParsedObject()
{
}

int ParsedAmbientLightObject::apply(RenderEngine & eng) const
{
  return eng.rd_ambient_light(color, intensity);
}

ParsedAmbientLightObject::~ParsedAmbientLightObject()
{
}

ParsedCatmullClarkSDSObject::
ParsedCatmullClarkSDSObject(const string & vt, 
			    const float * c, int nv, const int * vl, int nf, 
			    const int * cl, int nc, const float *s)
{
  vtype = vt;
  nvertex = nv;

  int vertex_array_size = get_vertex_size(vtype) * nv;
  
  coord = new float[vertex_array_size];
  
  memcpy(coord, c, vertex_array_size * sizeof(float));

  // Need to count size of other arrays -- majorly inefficient design

  int list_array_size = 0; 
  int i;

  nface = nf;

  // Vertex array
  for(i = 0; i < nface;)
    {
      if(vl[list_array_size] == -1)
	i++;
      list_array_size++;
    }

  vertex_list = new int[list_array_size];
  memcpy(vertex_list, vl, list_array_size * sizeof(int));

  // Crease lists
  
  ncrease = nc;
  list_array_size = 0;

  for(i = 0; i < ncrease;)
    {
      if(cl[list_array_size] == -1)
	i++;
      list_array_size++;
    }

  crease_list = new int[list_array_size];
  memcpy(crease_list, cl, list_array_size * sizeof(int));

  list_array_size -= 2 * ncrease; // count edges, not vertices; discard -1's

  sharpness = new float[list_array_size];
  memcpy(sharpness, s, list_array_size * sizeof(float));
  
}

int ParsedCatmullClarkSDSObject::apply(RenderEngine & eng) const
{
  return eng.rd_catmull_clark_sds(vtype, coord, nvertex, vertex_list, nface,
				  crease_list, ncrease, sharpness);
}

ParsedCatmullClarkSDSObject::~ParsedCatmullClarkSDSObject()
{
  delete [] coord;
  delete [] vertex_list;
  delete [] crease_list;
  delete [] sharpness;
}

int ParsedColorObject::apply(RenderEngine & eng) const
{
  return eng.rd_color(color);
}

ParsedColorObject::~ParsedColorObject()
{
}

int ParsedConeObject::apply(RenderEngine & eng) const
{
  return eng.rd_cone(height, radius, thetamax);
}

ParsedConeObject::~ParsedConeObject()
{
}

int ParsedCubeObject::apply(RenderEngine & eng) const
{
  return eng.rd_cube();
}

ParsedCubeObject::~ParsedCubeObject()
{
}

int ParsedCylinderObject::apply(RenderEngine & eng) const
{
  return eng.rd_cylinder(radius, zmin, zmax, thetamax);
}

ParsedCylinderObject::~ParsedCylinderObject()
{
}

int ParsedDiskObject::apply(RenderEngine & eng) const
{
  return eng.rd_disk(height, radius, thetamax);
}

ParsedDiskObject::~ParsedDiskObject()
{
}

int ParsedEmissionObject::apply(RenderEngine & eng) const
{
  return eng.rd_emission(color, intensity);
}

ParsedEmissionObject::~ParsedEmissionObject()
{
}

int ParsedHyperboloidObject::apply(RenderEngine & eng) const
{
  return eng.rd_hyperboloid(start, end, thetamax);
}

ParsedHyperboloidObject::~ParsedHyperboloidObject()
{
}

int ParsedKaObject::apply(RenderEngine & eng) const
{
  return eng.rd_k_ambient(Ka);
}

ParsedKaObject::~ParsedKaObject()
{
}

int ParsedKdObject::apply(RenderEngine & eng) const
{
  return eng.rd_k_diffuse(Kd);
}

ParsedKdObject::~ParsedKdObject()
{
}

int ParsedKeObject::apply(RenderEngine & eng) const
{
  return eng.rd_k_emission(Ke);
}

ParsedKeObject::~ParsedKeObject()
{
}

int ParsedKsObject::apply(RenderEngine & eng) const
{
  return eng.rd_k_specular(Ks);
}

ParsedKsObject::~ParsedKsObject()
{
}

int ParsedLineObject::apply(RenderEngine & eng) const
{
  return eng.rd_line(start, end);
}

ParsedLineObject::~ParsedLineObject()
{
}

ParsedLinesetObject::ParsedLinesetObject(const string & vt, int nv, float *c, 
					 int ns, int * vl)
{
  vtype = vt;
  nvertex = nv;

  int vertex_array_size = get_vertex_size(vtype) * nv;
  coord = new float[vertex_array_size];

  memcpy(coord, c, vertex_array_size * sizeof(float));

  nseg = ns; 

  // Need to determine size of vertex_list array -- yes this is inefficient
  // design

  int list_array_size = 0;
  int i = 0;
  while(i < nseg)
    {
      if(vl[list_array_size] == -1)
	i++;

      list_array_size++;
    }

  vertex_list = new int[list_array_size];

  memcpy(vertex_list, vl, list_array_size * sizeof(int));
}

int ParsedLinesetObject::apply(RenderEngine & eng) const
{
  return eng.rd_lineset(vtype.c_str(), nvertex, coord, nseg, vertex_list);
}

ParsedLinesetObject::~ParsedLinesetObject()
{
  delete [] vertex_list;
  delete [] coord;
}

int ParsedMapObject::apply(RenderEngine & eng) const
{
  return eng.rd_map(map_type.c_str(), label.c_str());
}

ParsedMapObject::~ParsedMapObject()
{
}

int ParsedMapBorderObject::apply(RenderEngine & eng) const
{
  return eng.rd_map_border(map_type.c_str(), horiz.c_str(), vert.c_str());
}

ParsedMapBorderObject::~ParsedMapBorderObject()
{
}

int ParsedMapBoundObject::apply(RenderEngine & eng) const
{
  return eng.rd_map_bound(map_type, smin, tmin, smax, tmax);
}

ParsedMapBoundObject::~ParsedMapBoundObject()
{
}

int ParsedMapLoadObject::apply(RenderEngine & eng) const
{
  return eng.rd_map_load(file.c_str(), label.c_str());
}

ParsedMapLoadObject::~ParsedMapLoadObject()
{
}

int ParsedMapSampleObject::apply(RenderEngine & eng) const
{
  return eng.rd_map_sample(map_type.c_str(), intra.c_str(), inter.c_str());
}

ParsedMapSampleObject::~ParsedMapSampleObject()
{
}

int ParsedOpacityObject::apply(RenderEngine & eng) const
{
  return eng.rd_opacity(opacity);
}

ParsedOpacityObject::~ParsedOpacityObject()
{
}

ParsedOptionArrayObject::ParsedOptionArrayObject(const string & s, int count,
						 const float * val)
{
  name = s;
  n = count;
  value = new float[n];

  memcpy(value, val, n * sizeof(float));
}

int ParsedOptionArrayObject::apply(RenderEngine & eng) const
{
  return eng.rd_option_array(name.c_str(), n, value);
}

ParsedOptionArrayObject::~ParsedOptionArrayObject()
{
  delete [] value;
}

int ParsedOptionBoolObject::apply(RenderEngine & eng) const
{
  return eng.rd_option_bool(name.c_str(), flag);
}

ParsedOptionBoolObject::~ParsedOptionBoolObject()
{
}

ParsedOptionListObject::ParsedOptionListObject(const string & s, int count,
						 const string * val)
{
  name = s;
  n = count;
  value = new string[n];

  for(int i = 0; i < n; i++)
    value[i] = val[i];
}

int ParsedOptionListObject::apply(RenderEngine & eng) const
{
  return eng.rd_option_list(name.c_str(), n, value);
}

ParsedOptionListObject::~ParsedOptionListObject()
{
  delete [] value;
}

int ParsedOptionRealObject::apply(RenderEngine & eng) const
{
  return eng.rd_option_real(name.c_str(), value);
}

ParsedOptionRealObject::~ParsedOptionRealObject()
{
}

int ParsedOptionStringObject::apply(RenderEngine & eng) const
{
  return eng.rd_option_string(name.c_str(), value);
}

ParsedOptionStringObject::~ParsedOptionStringObject()
{
}

int ParsedParaboloidObject::apply(RenderEngine & eng) const
{
  return eng.rd_paraboloid(rmax, zmin, zmax, thetamax);
}

ParsedParaboloidObject::~ParsedParaboloidObject()
{
}

ParsedBezierCurveObject::ParsedBezierCurveObject(const string & vt, 
						 int d, float * c)
{
  vtype = vt;
  deg = d;

  int vertex_array_size = get_vertex_size(vtype) * (d+1);

  coord = new float[vertex_array_size];

  memcpy(coord, c, vertex_array_size * sizeof(float));
}

int ParsedBezierCurveObject::apply(RenderEngine & eng) const
{
  return eng.rd_bezier_curve(vtype.c_str(), deg, coord);
}

ParsedBezierCurveObject::~ParsedBezierCurveObject()
{
}

ParsedBezierPatchObject::ParsedBezierPatchObject(const string & vt, 
						 int ud, int vd, float * c)
{
  vtype = vt;
  u_deg = ud;
  v_deg = vd;

  int vertex_array_size = get_vertex_size(vtype) * (ud+1)*(vd+1);

  coord = new float[vertex_array_size];

  memcpy(coord, c, vertex_array_size * sizeof(float));
}

int ParsedBezierPatchObject::apply(RenderEngine & eng) const
{
  return eng.rd_bezier_patch(vtype.c_str(), u_deg, v_deg, coord);
}

ParsedBezierPatchObject::~ParsedBezierPatchObject()
{
}

int ParsedPointObject::apply(RenderEngine & eng) const
{
  return eng.rd_point(p);
}

ParsedPointObject::~ParsedPointObject()
{
}

ParsedPointsetObject::ParsedPointsetObject(const string & vt, int np, float *c)
{
  vtype = vt;
  npoints = np;
  
  int point_array_size = get_vertex_size(vtype) * np;

  coord = new float[point_array_size];

  memcpy(coord, c, point_array_size * sizeof(float));

}

int ParsedPointsetObject::apply(RenderEngine & eng) const
{
  return eng.rd_pointset(vtype.c_str(), npoints, coord);
}

ParsedPointsetObject::~ParsedPointsetObject()
{
  delete [] coord;
}

ParsedPolysetObject::ParsedPolysetObject(const string & vt, int nv, float *c, 
					 int nf, int * vl)
{
  vtype = vt;
  nvertex = nv;

  int vertex_array_size = get_vertex_size(vtype) * nv;
  coord = new float[vertex_array_size];

  memcpy(coord, c, vertex_array_size * sizeof(float));

  nface = nf; 

  // Need to determine size of vertex_list array -- yes this is inefficient
  // design

  int list_array_size = 0;
  int i = 0;
  while(i < nface)
    {
      if(vl[list_array_size] == -1)
	i++;

      list_array_size++;
    }

  vertex_list = new int[list_array_size];

  memcpy(vertex_list, vl, list_array_size * sizeof(int));
}

int ParsedPolysetObject::apply(RenderEngine & eng) const
{
  return eng.rd_polyset(vtype.c_str(), nvertex, coord, nface, vertex_list);
}

ParsedPolysetObject::~ParsedPolysetObject()
{
  delete [] vertex_list;
  delete [] coord;
}

int ParsedRotateXYObject::apply(RenderEngine & eng) const
{
  return eng.rd_rotate_xy(theta);
}

ParsedRotateXYObject::~ParsedRotateXYObject()
{
}

int ParsedRotateZXObject::apply(RenderEngine & eng) const
{
  return eng.rd_rotate_zx(theta);
}

ParsedRotateZXObject::~ParsedRotateZXObject()
{
}

int ParsedRotateYZObject::apply(RenderEngine & eng) const
{
  return eng.rd_rotate_yz(theta);
}

ParsedRotateYZObject::~ParsedRotateYZObject()
{
}

int ParsedScaleObject::apply(RenderEngine & eng) const
{
  return eng.rd_scale(scale);
}

ParsedScaleObject::~ParsedScaleObject()
{
}

int ParsedSpecularObject::apply(RenderEngine & eng) const
{
  return eng.rd_specular_color(color, exponent);
}

ParsedSpecularObject::~ParsedSpecularObject()
{
}

int ParsedSphereObject::apply(RenderEngine & eng) const
{
  return eng.rd_sphere(radius, zmin, zmax, thetamax);
}

ParsedSphereObject::~ParsedSphereObject()
{
}

int ParsedSqSphereObject::apply(RenderEngine & eng) const
{
  return eng.rd_sqsphere(radius, north, east, zmin, zmax, thetamax);
}

ParsedSqSphereObject::~ParsedSqSphereObject()
{
}

int ParsedSqTorusObject::apply(RenderEngine & eng) const
{
  return eng.rd_sqtorus(radius1, radius2, north, east, 
			phimin, phimax, thetamax);
}

ParsedSqTorusObject::~ParsedSqTorusObject()
{
}

int ParsedSurfaceObject::apply(RenderEngine & eng) const
{
  return eng.rd_surface(surf.c_str());
}

ParsedSurfaceObject::~ParsedSurfaceObject()
{
}

int ParsedTranslateObject::apply(RenderEngine & eng) const
{
  return eng.rd_translate(trans);
}

ParsedTranslateObject::~ParsedTranslateObject()
{
}

int ParsedTorusObject::apply(RenderEngine & eng) const
{
  return eng.rd_torus(radius1, radius2, phimin, phimax, thetamax);
}

ParsedTorusObject::~ParsedTorusObject()
{
}

int ParsedTubeObject::apply(RenderEngine & eng) const
{
  return eng.rd_tube(start, end, radius);
}

ParsedTubeObject::~ParsedTubeObject()
{
}

int ParsedXformPopObject::apply(RenderEngine & eng) const
{
  return eng.rd_xform_pop();
}

ParsedXformPopObject::~ParsedXformPopObject()
{
}

int ParsedXformPushObject::apply(RenderEngine & eng) const
{
  return eng.rd_xform_push();
}

ParsedXformPushObject::~ParsedXformPushObject()
{
}
