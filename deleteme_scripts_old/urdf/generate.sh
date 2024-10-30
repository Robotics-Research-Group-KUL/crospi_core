# use source to run this file
xacro /opt/ros/humble/share/ur_description/urdf/ur.urdf.xacro name:=ur10e ur_type:=ur10e > ur10e.urdf
urdf_to_wm ur10e.urdf 


