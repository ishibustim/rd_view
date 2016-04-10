Display "Colored Mirrors" "Screen" "rgbobject"

Format 960 480

CameraAt  0 0 10
CameraEye -30 -40 15
CameraUp  0 0 1
CameraFOV 30.0

Background 0.28 0.56 0.71

WorldBegin
AmbientLight 1.0 1.0 1.0 0.39
PointLight 0 0 100 1.0 1.0 1.0 3900
PointLight -10 10 20 1.0 0.0 0.0 177
PointLight  10 10 20 0.0 1.0 0.0 177
PointLight  10 -10 20 0.0 0.0 1.0 177

Surface "plastic"
Specular 1.0 1.0 1.0 20

#semi-glossy white plastic ground plane

Ka 0.7
Kd 0.7
Ks 0.3

PolySet "P"
4 1
-100.0 -100.0 0.0
 100.0 -100.0 0.0
 100.0  100.0 0.0
-100.0  100.0 0.0

0 1 2 3 -1

XformPush
Translate 0 0 5
Sphere 5 -5 5 360
XformPop

# A red mirror

Color 1.0 0.1 0.1
Specular 1.0 0.1 0.1 50
Ka 0.2
Kd 0.2
Ks 0.8

PolySet "P"
4 1
-10 20  0
 15 20  0
 15 20 20
-10 20 20

0 1 2 3 -1

# A blue mirror

Color 0.1 0.1 1.0
Specular 0.1 0.1 1.0 50
Ka 0.2
Kd 0.2
Ks 0.8

PolySet "P"
4 1
 20  15  0
 20 -10  0
 20 -10 20
 20  15 20 

0 1 2 3 -1

# A green mirror

Color 0.1 1.0 0.1
Specular 0.1 1.0 0.1 50
Ka 0.2
Kd 0.2
Ks 0.8

PolySet "P"
4 1
 14 20  0
 20 14  0
 20 14 20
 14 20 20

0 1 2 3 -1

WorldEnd

