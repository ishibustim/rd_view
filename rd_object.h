#ifndef RD_PARSE_OBJECT
#define RD_PARSE_OBJECT

// This is the whole menagerie of RD parsed objects
// Placed in one file for easy access
#include "rd_enginebase.h"
#include <string>
using std::string;

class ParsedObject
{
public:
  virtual int apply(RenderEngine &) const = 0; // const ? 
  // Applies this object to the given engine.  That may mean storing it or
  // rendering it or ignoring it.

  virtual ~ParsedObject() = 0;
};

class ParsedAmbientLightObject : public ParsedObject
{
  float color[3]; 
  double intensity;
public:
  ParsedAmbientLightObject(float c[3], double i)
    : intensity(i) {color[0] = c[0]; color[1] = c[1]; color[2] = c[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedAmbientLightObject();
};

class ParsedCatmullClarkSDSObject : public ParsedObject
{
  string vtype;
  int nvertex;
  float * coord;
  // This may change.  It may be better to have vertex / edge / face
  // representation.
  int nface;
  int * vertex_list;
  int ncrease;
  int * crease_list;
  float * sharpness;

public:
  ParsedCatmullClarkSDSObject(const string & vt, const float * c, int nv,
			      const int * vl, int nf, const int * cl, int nc,
			      const float *s);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedCatmullClarkSDSObject();
  
};

class ParsedColorObject : public ParsedObject
{
  float color[3];
public:
  ParsedColorObject(const float c[])
  {color[0] = c[0]; color[1] = c[1]; color[2] = c[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedColorObject();
};

class ParsedConeObject : public ParsedObject
{
  float height, radius, thetamax;
public:
  ParsedConeObject(float h, float r, float t)
  : height(h), radius(r), thetamax(t){}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedConeObject();
};

class ParsedCubeObject : public ParsedObject
{
public:
  ParsedCubeObject() {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedCubeObject();
};

class ParsedCylinderObject : public ParsedObject
{
  float radius, zmin, zmax, thetamax;
public:
  ParsedCylinderObject(float r, float z1, float z2, float t) :
    radius(r), zmin(z1), zmax(z2), thetamax(t) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedCylinderObject();
};

class ParsedDiskObject : public ParsedObject
{
  float height, radius, thetamax;
public:
  ParsedDiskObject(float h, float r, float t) :
    height(h), radius(r), thetamax(t) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedDiskObject();
};

class ParsedEmissionObject : public ParsedObject
{
  float color[3];
  float intensity;
public:
  ParsedEmissionObject(float c[3], float i)
  {color[0] = c[0]; color[1] = c[1]; color[2] = c[2]; intensity = i;}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedEmissionObject();
};

class ParsedHyperboloidObject : public ParsedObject
{
  float start[3], end[3], thetamax;
public:
  ParsedHyperboloidObject(float s[3], float e[3], float t)
    : thetamax(t){start[0] = s[0]; start[1] = s[1]; start[2] = s[2];
    end[0] = e[0]; end[1] = e[1]; end[2] = e[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedHyperboloidObject();
};

class ParsedKaObject : public ParsedObject
{
  float Ka;
public:
  ParsedKaObject(float k) : Ka(k) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedKaObject();
};

class ParsedKdObject : public ParsedObject
{
  float Kd;
public:
  ParsedKdObject(float k) : Kd(k) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedKdObject();
};

class ParsedKeObject : public ParsedObject
{
  float Ke;
public:
  ParsedKeObject(float k) : Ke(k) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedKeObject();
};

class ParsedKsObject : public ParsedObject
{
  float Ks;
public:
  ParsedKsObject(float k) : Ks(k) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedKsObject();
};

class ParsedLineObject : public ParsedObject
{
  float start[3], end[3];
public:
  ParsedLineObject(float s[3], float e[3])
  {start[0] = s[0]; start[1] = s[1]; start[2] = s[2];
  end[0] = e[0]; end[1] = e[1]; end[2] = e[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedLineObject();
};

class ParsedLinesetObject : public ParsedObject
{
  string vtype;
  int nvertex;
  float * coord;
  int nseg;
  int * vertex_list;

public:
  ParsedLinesetObject(const string & vt, int nv, float * c, int ns, int * vl);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedLinesetObject();
};

class ParsedMapObject : public ParsedObject
{
  string map_type, label;
public:
  ParsedMapObject(const string & s1, const string & s2) 
    : map_type(s1), label(s2) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedMapObject();
};

class ParsedMapBorderObject : public ParsedObject
{
  string map_type, horiz, vert;
public:
 ParsedMapBorderObject(const string&  t1,
		       const string & s1, const string & s2) 
   : map_type(t1), horiz(s1), vert(s2) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedMapBorderObject();
};

class ParsedMapBoundObject : public ParsedObject
{
  string map_type;
  float smin, tmin, smax, tmax;
public:
 ParsedMapBoundObject(const string & m1,
			  float f1, float f2, float f3, float f4)
   : map_type(m1), smin(f1), tmin(f2), smax(f3), tmax(f4) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedMapBoundObject();
};

class ParsedMapLoadObject : public ParsedObject
{
  string file, label;

public:
  ParsedMapLoadObject(const string & s1, const string & s2)
    : file(s1), label(s2) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedMapLoadObject();
};

class ParsedMapSampleObject : public ParsedObject
{
  string map_type, intra, inter;

public:
  ParsedMapSampleObject(const string & m1, const string & s1, const string & s2)
    : map_type(m1), intra(s1), inter(s2) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedMapSampleObject();
};

class ParsedOpacityObject : public ParsedObject
{
  float opacity;
public:
  ParsedOpacityObject(float k) : opacity(k) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOpacityObject();
};

class ParsedOptionArrayObject : public ParsedObject
{
  string name;
  int n;
  float * value;
public:
  ParsedOptionArrayObject(const string & s, int count, const float * val);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOptionArrayObject();
};

class ParsedOptionBoolObject : public ParsedObject
{
  string name;
  bool flag;
public:
  ParsedOptionBoolObject(const string & s, bool f)
    : name(s), flag(f) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOptionBoolObject();
};

class ParsedOptionListObject : public ParsedObject
{
  string name;
  int n;
  string * value;
  const char * * list;
public:
  ParsedOptionListObject(const string & s, int n, const string * val);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOptionListObject();
};

class ParsedOptionRealObject : public ParsedObject
{
  string name;
  float value;
public:
  ParsedOptionRealObject(const string & s, float f)
    : name(s), value(f) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOptionRealObject();
};

class ParsedOptionStringObject : public ParsedObject
{
  string name;
  string value;
public:
  ParsedOptionStringObject(const string & s, const string & v)
    : name(s), value(v) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedOptionStringObject();
};

class ParsedParaboloidObject : public ParsedObject
{
  float rmax, zmin, zmax, thetamax;
public:
  ParsedParaboloidObject(float r, float z1, float z2, float t) :
    rmax(r), zmin(z1), zmax(z2), thetamax(t) {}
  
  virtual int apply(RenderEngine &) const;

  virtual ~ParsedParaboloidObject();
};

class ParsedBezierCurveObject : public ParsedObject
{
  string vtype;
  int deg;
  float * coord;

public:
  ParsedBezierCurveObject(const string & vt, int d, float * c);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedBezierCurveObject();
};


class ParsedBezierPatchObject : public ParsedObject
{
  string vtype;
  int u_deg, v_deg;
  float * coord;

public:
  ParsedBezierPatchObject(const string & vt, int ud, int vd, float * c);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedBezierPatchObject();
};

class ParsedPointObject : public ParsedObject
{
  float p[3];
public:
  ParsedPointObject(float p1[3])
  {p[0] = p1[0]; p[1] = p1[1]; p[2] = p1[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedPointObject();
};

class ParsedPointsetObject : public ParsedObject
{
  string vtype;
  int npoints;
  float * coord;

public:
  ParsedPointsetObject(const string & vt, int np, float * c);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedPointsetObject();
};

class ParsedPolysetObject : public ParsedObject
{
  string vtype;
  int nvertex;
  float * coord;
  int nface;
  int * vertex_list;

public:
  ParsedPolysetObject(const string & vt, int nv, float * c, int nf, int * vl);

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedPolysetObject();
};

class ParsedRotateXYObject : public ParsedObject
{
  float theta;
public:
  ParsedRotateXYObject(float f) : theta(f) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedRotateXYObject();
};

class ParsedRotateZXObject : public ParsedObject
{
  float theta;
public:
  ParsedRotateZXObject(float f) : theta(f) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedRotateZXObject();
};

class ParsedRotateYZObject : public ParsedObject
{
  float theta;
public:
  ParsedRotateYZObject(float f) : theta(f) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedRotateYZObject();
};

class ParsedScaleObject : public ParsedObject
{
  float scale [3];
public:
  ParsedScaleObject(const float s[3])
  {scale[0] = s[0]; scale[1] = s[1]; scale[2] = s[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedScaleObject();
};

class ParsedSpecularObject : public ParsedObject
{
  float color[3];
  int exponent;
public:
  ParsedSpecularObject(const float c[3], int e)
    : exponent(e) {color[0] = c[0]; color[1] = c[1]; color[2] = c[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedSpecularObject();
};

class ParsedSphereObject : public ParsedObject
{
  double radius, zmin, zmax, thetamax;
public:
  ParsedSphereObject(double r, double z1, double z2, double t)
    : radius(r), zmin(z1), zmax(z2), thetamax(t) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedSphereObject();
};

class ParsedSqSphereObject : public ParsedObject
{
  double radius, north, east, zmin, zmax, thetamax;
public:
  ParsedSqSphereObject(double r, double n, double e,
		       double z1, double z2, double t)
    : radius(r), north(n), east(e), zmin(z1), zmax(z2), thetamax(t) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedSqSphereObject();
};

class ParsedSqTorusObject : public ParsedObject
{
  double radius1, radius2, north, east, phimin, phimax, thetamax;
public:
  ParsedSqTorusObject(double r1, double r2, double n, double e,
		       double p1, double p2, double t)
    : radius1(r1), radius2(r2), north(n), east(e), 
    phimin(p1), phimax(p2), thetamax(t) {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedSqTorusObject();
};

class ParsedSurfaceObject : public ParsedObject
{
  string surf;
public:
  ParsedSurfaceObject(const string & s) : surf(s){}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedSurfaceObject();
};

class ParsedTorusObject : public ParsedObject
{
  float radius1, radius2, phimin, phimax, thetamax;
public:
  ParsedTorusObject(float r1, float r2, float p1, float p2, float t) 
    : radius1(r1), radius2(r2), phimin(p1), phimax(p2), thetamax(t) {}

  virtual int apply (RenderEngine &) const;

  virtual ~ParsedTorusObject();
};

class ParsedTranslateObject : public ParsedObject
{
  float trans[3];
public:
  ParsedTranslateObject(const float t[3])
  {trans[0] = t[0]; trans[1] = t[1]; trans[2] = t[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedTranslateObject();
};

class ParsedTubeObject : public ParsedObject
{
  float start[3], end[3], radius;
public:
  ParsedTubeObject(const float s[3], const float e[3], float r)
    : radius(r) {start[0] = s[0]; start[1] = s[1]; start[2] = s[2];
    end[0] = e[0]; end[1] = e[1]; end[2] = e[2];}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedTubeObject();
};

class ParsedXformPopObject : public ParsedObject
{
public:
  ParsedXformPopObject() {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedXformPopObject();
};

class ParsedXformPushObject : public ParsedObject
{
public:
  ParsedXformPushObject() {}

  virtual int apply(RenderEngine &) const;

  virtual ~ParsedXformPushObject();
};

#endif /* RD_BASE_PARSE_OBJECT */
