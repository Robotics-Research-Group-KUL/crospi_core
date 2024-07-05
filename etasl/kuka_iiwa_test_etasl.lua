require("context")
require("geometric")
worldmodel=require("worldmodel")
urdfreader=require("urdfreader")
require("math")

--
-- read robot model:
--

xmlstr = urdfreader.loadFile("/workspaces/colcon_ws/src/etasl_ros2/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
robot:writeDot("ur10_robot.dot")
VL = {}
rv = robot:getExpressions(VL,ctx,{ee={'tool0','base_link'}})
ee = rv['ee']
robot_joints={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}

-- ======================================== FRAMES ========================================

tf = ee



ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tcp",coord_x(origin(tf)))
ctx:setOutputExpression("y_tcp",coord_y(origin(tf)))
ctx:setOutputExpression("z_tcp",coord_z(origin(tf)))
ctx:setOutputExpression("tf",tf)




-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(tf)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(tf)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(tf)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(tf))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
