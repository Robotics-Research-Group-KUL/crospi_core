require("context")
require("geometric")
worldmodel=require("worldmodel")
urdfreader=require("urdfreader")
require("math")

--
-- read robot model:
--

xmlstr = urdfreader.loadFile("/workspaces/colcon_ws/src/etasl_ros2/robot_description/urdf/kuka_iiwa/use_case_setup_iiwa.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
robot:writeDot("kuka_iiwa_robot.dot")
VL = {}
rv = robot:getExpressions(VL,ctx,{ee={'right_tool0','world'}})
ee = rv['ee']
robot_joints={"right_joint_a1","right_joint_a2","right_joint_a3","right_joint_a4","right_joint_a5","right_joint_a6","right_joint_a7"}

-- ======================================== FRAMES ========================================

tf = ee

last_joint   = ctx:getScalarExpr(robot_joints[7])

Constraint{
    context=ctx,
    name="joint_trajectory",
    expr= last_joint - sin(time) * 10*3.1416/180,
    priority = 2,
    K=4
};


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
