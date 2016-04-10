# This is a test/example rd file

Display "Depth Test"  "Screen" "rgbsingle"
Format 640 480

# This is an embedded object version of the Wireframe Gallery

ObjectBegin "Pedestal"

XformPush # Pedestal Base
Scale 1 1 0.1
Translate 0 0 1
Cube
XformPop # Pedestal Base

# Column
XformPush
Translate 0 0 0.2
Scale 0.5 0.5 4
Cylinder 1.0 0.0 1.0 360.0
XformPop

XformPush # Pedestal Top
Translate 0 0 4.2
Scale 1 1 0.1
Translate 0 0 1
Cube
XformPop # Pedestal Top

ObjectEnd

ObjectBegin "Tetrahedron"
# Tetrahedron
PolySet "P"
4 # Vertices
4 # Faces

# Vertex points
 1.0  1.0  1.0
 1.0 -1.0 -1.0
-1.0  1.0 -1.0
-1.0 -1.0  1.0

# Face indices
3 2 1 -1
2 3 0 -1
1 0 3 -1
0 1 2 -1

ObjectEnd # Tetrahedron

ObjectBegin "Octahedron"

# Octahedron
PolySet "P"
6 # Vertices

8 # Faces

# Vertex points
 1.0  0.0  0.0
-1.0  0.0  0.0
 0.0  1.0  0.0
 0.0 -1.0  0.0
 0.0  0.0  1.0
 0.0  0.0 -1.0

# Face indices
0 2 4 -1
2 0 5 -1
3 0 4 -1
0 3 5 -1
2 1 4 -1
1 2 5 -1
1 3 4 -1
3 1 5 -1

ObjectEnd # Octahedron

ObjectBegin "Icosahedron"
# Icosahedron
PolySet "P"
12 # Vertices
20 # Faces

# Vertex points
 1.618034  1.0  0.0
-1.618034  1.0  0.0
 1.618034 -1.0  0.0
-1.618034 -1.0  0.0

 1.0  0.0  1.618034
 1.0  0.0 -1.618034
-1.0  0.0  1.618034
-1.0  0.0 -1.618034

 0.0  1.618034  1.0
 0.0 -1.618034  1.0
 0.0  1.618034 -1.0
 0.0 -1.618034 -1.0

# Face indices
 0  8  4 -1
 0  5 10 -1
 2  4  9 -1
 2 11  5 -1
 1  6  8 -1
 1 10  7 -1
 3  9  6 -1
 3  7 11 -1

 0 10  8 -1
 1  8 10 -1
 2  9 11 -1
 3 11  9 -1

 4  2  0 -1
 5  0  2 -1
 6  1  3 -1
 7  3  1 -1

 8  6  4 -1
 9  4  6 -1
10  5  7 -1
11  7  5 -1

ObjectEnd # Icosahedron

ObjectBegin "Dodecahedron"
# Dodecahedron
PolySet "P"
20 # Vertices
12 # Faces

# Vertex points
 1.0  1.0  1.0
 1.0  1.0 -1.0
 1.0 -1.0  1.0
 1.0 -1.0 -1.0
-1.0  1.0  1.0
-1.0  1.0 -1.0
-1.0 -1.0  1.0
-1.0 -1.0 -1.0

 0.618034  1.618034  0.0
-0.618034  1.618034  0.0
 0.618034 -1.618034  0.0
-0.618034 -1.618034  0.0

 1.618034  0.0  0.618034
 1.618034  0.0 -0.618034
-1.618034  0.0  0.618034
-1.618034  0.0 -0.618034

 0.0  0.618034  1.618034
 0.0 -0.618034  1.618034
 0.0  0.618034 -1.618034
 0.0 -0.618034 -1.618034


# Face indices
 1  8  0 12 13 -1
 4  9  5 15 14 -1
 2 10  3 13 12 -1
 7 11  6 14 15 -1

 2 12  0 16 17 -1
 1 13  3 19 18 -1
 4 14  6 17 16 -1
 7 15  5 18 19 -1

 4 16  0  8  9 -1
 2 17  6 11 10 -1
 1 18  5  9  8 -1
 7 19  3 10 11 -1

ObjectEnd # Dodecahdron


FrameBegin 1

CameraEye   -25.0 -30.0 20.0
CameraAt    0.0 0.0 3.0
CameraUp   0 0 1
CameraFOV  30.0

WorldBegin

XformPush # Sphere on Pedestal

Rotate "Z" 0
Translate 0 10 0

Color 1.0 0.0 0.0

XformPush
Translate 0 0 6
Sphere 1.0 -1.0 1.0 360.0
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Sphere on Pedestal

XformPush # Cube on Pedestal
Rotate "Z" 51.43
Translate 0 10 0

XformPush
Color 0.0 0.0 1.0
Translate 0 0 6.5
Rotate "X" 35.26
Rotate "Y" 45
Cube
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Cube on Pedestal

XformPush # Cone on Pedestal
Rotate "Z" 102.86
Translate 0 10 0

XformPush
Color 0.0 1.0 0.0
Translate 0 0 5
Scale 1 1 2
Cone 1.0 1.0 360.0
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Cone on Pedestal

XformPush # Tetrahedron on Pedestal
Rotate "Z" 154.29
Translate 0 10 0

XformPush
Color 0.5 0.3 0.0
Translate 0 0 6
Rotate "X" -35.2644
Rotate "Y" 45

ObjectInstance "Tetrahedron"
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Tetrahedron on Pedestal

XformPush # Octahedron on Pedestal
Rotate "Z" 205.71
Translate 0 10 0

XformPush
Color 0.7 0.0 0.8

Translate 0 0 6
Rotate "X" -35.2644
Rotate "Y" 45

ObjectInstance "Octahedron"
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Octahedron on Pedestal

XformPush # Icosahedron on Pedestal
Rotate "Z" 257.14
Translate 0 10 0

XformPush
Color 1.0 0.7 0.7
Translate 0 0 6
Rotate "X" -20.9051

ObjectInstance "Icosahedron"
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Icosahedron on Pedestal

XformPush # Dodecahedron on Pedestal
Rotate "Z" 308.57
Translate 0 10 0

XformPush
Color 1.0 1.0 0.0
Translate 0 0 6
Rotate "X" -58.2825

ObjectInstance "Dodecahedron"
XformPop

Color 0.5 0.5 0.5

ObjectInstance "Pedestal"

XformPop  # Dodecahedron on Pedestal

XformPush # Star Pattern on Floor

Scale 8.5 8.5 1
Color 1.0 0.7 0.3
Line  0.000  1.000 0  0.975 -0.223 0
Line  0.975 -0.223 0 -0.434 -0.901 0
Line -0.434 -0.901 0 -0.782  0.623 0
Line -0.782  0.623 0  0.782  0.623 0
Line  0.782  0.623 0  0.434 -0.901 0
Line  0.434 -0.901 0 -0.975 -0.223 0
Line -0.975 -0.233 0  0.000  1.000 0

XformPop

WorldEnd
FrameEnd






