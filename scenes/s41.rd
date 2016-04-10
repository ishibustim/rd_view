# This is a test/example rd file
# Two spheres, pure diffuse surface, one distant light behind eye.


Display "Simple Shading"  "Screen" "rgbsingle"
Format 640 480
CameraFOV 5
CameraEye 0 0 50

WorldBegin

FarLight 0 0 -1  1.0  1.0  1.0  1.0

Ka 0.0
Kd 1.0

Translate 1.25 0 0

Sphere 1.0 -1.0 1.0 360.0

Translate -2.5 0 0 

Sphere 1.0 -1.0 1.0 360.0

WorldEnd
