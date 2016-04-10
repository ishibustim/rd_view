Display "Sphere 1" "Screen" "rgbsingle"
Format 200 200
Background 0 0 0 

WorldBegin

AmbientLight 1.0 1.0 1.0 0.3921

PointLight -5 5 10 1.0 1.0 1.0 785

#Spheres

Surface "plastic"
Ka 0.3
Kd 0.3
Ks 0.7
Specular 1.0 1.0 1.0 50

XformPush
Translate 0 0 -10
Sphere 3 -3 3 360
XformPop

XformPush
Ka 0.5
Kd 0.5
Ks 0.5
Translate 5 5 -100
Sphere 50 -50 50 360
XformPop

XformPush
Translate -10 -5 -30
Sphere 2 -2 2 360
XformPop

WorldEnd
