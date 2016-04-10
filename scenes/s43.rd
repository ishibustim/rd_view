# This is a test/example rd file
# Two diffuse spheres, three colored directional lights
# One point light between the spheres.

Display "Point Light Test"  "Screen" "rgbsingle"
Format 640 480
CameraFOV 5
CameraEye 0 0 50

WorldBegin

#FarLight 0 0 -1  1.0  1.0  1.0  1.0

FarLight  1  1 -0.5  1.0 0.0 0.0  0.5  #Red Light    -- Lower Left
FarLight  0 -1 -0.5  0.0 1.0 0.0  0.5  #Green Light  -- Up
FarLight -1  1 -0.5  0.0 0.0 1.0  0.5  #Blue Light   -- Lower Right

PointLight 0 0 0  1.0 1.0 1.0 1.0  # White light -- centered

Ka 0.0
Kd 1.0

Translate -1.25 0 0

Sphere 1.0 -1.0 1.0 360.0

Translate 2.5 0 0 

OptionBool "Interpolate" off

Sphere 1.0 -1.0 1.0 360.0

WorldEnd
