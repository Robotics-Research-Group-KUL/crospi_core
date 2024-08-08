# obsolete

#!/usr/bin/python3
from ament_index_python import get_package_share_directory
from os.path import join
from os import system

xacrofile = join(get_package_share_directory("ur_description"), "urdf","ur.urdf.xacro")
urdffile  = join(get_package_share_directory("etasl_node"),"scripts","urdf","ur10.urdf")
print("xacro file : ",xacrofile)
print("urdfs file : ",urdffile)
parameters = "ur_type:=ur10 name:=UR"
system('ros2 run xacro xacro '+xacrofile+' ' + parameters+ ' > ' + urdffile)

