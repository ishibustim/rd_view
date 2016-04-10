#Display "Hmmmm" "Screen" "rgbobject"
Display "Hmmmm" "Screen" "rgbsingle"

Format 600 600

CameraFOV 30.0

WorldBegin
AmbientLight 1.0 1.0 1.0 0.3921
PointLight -15 15 50 1.0 1.0 1.0 1650
PointLight 0 0 -140 1.0 1.0 1.0 400
FarLight -2 -5 -2 1.0 1.0 1.0 0.3921

PointLight -15 15 -50 1.0 0.0 0.0 4300
PointLight  0  15 -50 0.0 1.0 0.0 4300
PointLight  15 15 -50 0.0 0.0 1.0 4300

#Glossy white plastic
Surface "plastic"
Specular 1.0 1.0 1.0 50

Ka 0.6
Kd 0.6
Ks 0.4

XformPush
Translate 30 20 -160
Sphere 20 -20 20 360
XformPop

XformPush
Translate -30 20 -160
Sphere 20 -20 20 360
XformPop

# Glossy blue plastic
Color 0.0 0.0 1.0

XformPush
Translate 25 16 -155
Sphere 15 -15 15 360
XformPop

XformPush
Translate -25 16 -155
Sphere 15 -15 15 360
XformPop

# Glossy black plastic

Color 0.1 0.1 0.1
Specular 1.0 1.0 1.0 75

XformPush
Translate 20 13 -150
Sphere 8.7 -8.7 8.7 360
XformPop

XformPush
Translate -20 13 -150
Sphere 8.0 -8.7 8.7 360
XformPop

# Gold
Color 1.0 0.8 0.4
Surface "plastic"
Specular 1.0 0.8 0.4 25

Ka 0.3
Kd 0.2
Ks 0.8

XformPush
Translate 0 -20 -130
Sphere 15 -15 15 360
XformPop

WorldEnd

