# This is a test/example rd file
# Array of white spheres.  Increasing Ks from left to right.
# Increasing specular exponent from bottom to top.


Display "Surface Properties"  "Screen" "rgbsingle"
Format 640 480
CameraFOV 1.7
CameraEye 0 0 500

Background 0.5 0.5 0.8
Clipping 490 510

WorldBegin

# Lights
FarLight -1 -1 -1  1.0  1.0  1.0  1.0  # Upper right

#Surface Color
Color 1.0 1.0 1.0
Surface "plastic"
Ka 0.0

# Row 0 Bottom
Specular 1.0 1.0 1.0  1

Kd 1.0
Ks 0.0

XformPush
Translate -7.5 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.8333
Ks 0.1667

XformPush
Translate -5.0 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.6667
Ks 0.3333

XformPush
Translate -2.5 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.5
Ks 0.5

XformPush
Translate 0.0 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.3333
Ks 0.6667

XformPush
Translate 2.5 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.1667
Ks 0.8333

XformPush
Translate 5.0 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.0
Ks 1.0

XformPush
Translate 7.5 -5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

# Row 1 
Specular 1.0 1.0 1.0  3

Kd 1.0
Ks 0.0

XformPush
Translate -7.5 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.8333
Ks 0.1667

XformPush
Translate -5.0 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.6667
Ks 0.3333

XformPush
Translate -2.5 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.5
Ks 0.5

XformPush
Translate 0.0 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.3333
Ks 0.6667

XformPush
Translate 2.5 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.1667
Ks 0.8333

XformPush
Translate 5.0 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.0
Ks 1.0

XformPush
Translate 7.5 -2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

# Row 2
Specular 1.0 1.0 1.0  10

Kd 1.0
Ks 0.0

XformPush
Translate -7.5 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.8333
Ks 0.1667

XformPush
Translate -5.0 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.6667
Ks 0.3333

XformPush
Translate -2.5 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.5
Ks 0.5

XformPush
Translate 0.0 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.3333
Ks 0.6667

XformPush
Translate 2.5 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.1667
Ks 0.8333

XformPush
Translate 5.0 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.0
Ks 1.0

XformPush
Translate 7.5 0.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

# Row 3
Specular 1.0 1.0 1.0  32

Kd 1.0
Ks 0.0

XformPush
Translate -7.5 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.8333
Ks 0.1667

XformPush
Translate -5.0 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.6667
Ks 0.3333

XformPush
Translate -2.5 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.5
Ks 0.5

XformPush
Translate 0.0 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.3333
Ks 0.6667

XformPush
Translate 2.5 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.1667
Ks 0.8333

XformPush
Translate 5.0 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.0
Ks 1.0

XformPush
Translate 7.5 2.5 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

# Row 4 Top
Specular 1.0 1.0 1.0  100

Kd 1.0
Ks 0.0

XformPush
Translate -7.5 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.8333
Ks 0.1667

XformPush
Translate -5.0 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.6667
Ks 0.3333

XformPush
Translate -2.5 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.5
Ks 0.5

XformPush
Translate 0.0 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.3333
Ks 0.6667

XformPush
Translate 2.5 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.1667
Ks 0.8333

XformPush
Translate 5.0 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

Kd 0.0
Ks 1.0

XformPush
Translate 7.5 5.0 0
Sphere 1.0 -1.0 1.0 360.0
XformPop

WorldEnd
