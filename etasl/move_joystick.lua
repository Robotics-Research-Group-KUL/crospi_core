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

-- =============================== INITIAL POSE ==============================
K_joystick = 0.2
joystick_input   = ctx:createInputChannelTwist("joystick_input")
-- joystick_input = twist(vector(0,0,-0.05),vector(0,0,0))


-- =============================== INSTANTANEOUS FRAME ==============================

-- tf_inst = inv(make_constant(tf))*tf

desired_vel_x = coord_x(transvel(joystick_input)) 
desired_vel_y = coord_y(transvel(joystick_input)) 
desired_vel_z = coord_z(transvel(joystick_input)) 

desired_omega_x = coord_x(rotvel(joystick_input)) 
desired_omega_y = coord_y(rotvel(joystick_input)) 
desired_omega_z = coord_z(rotvel(joystick_input)) 

-- Translation velocities
Constraint{
    context = ctx,
    name    = "x_velocity",
    expr    = coord_x(origin(tf)),
    target  = desired_vel_x*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_velocity",
    expr    = coord_y(origin(tf)),
    target  = desired_vel_y*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_velocity",
    expr    = coord_z(origin(tf)),
	target  = desired_vel_z*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

-- Orientation velocities
Constraint{
    context = ctx,
    name    = "x_angular",
    expr    = coord_x(getRotVec(rotation(tf))) - desired_omega_x*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "y_angular",
    expr    = coord_y(getRotVec(rotation(tf))) - desired_omega_y*time,
    K       = 0,
    weight  = 1,
    priority= 2
};

Constraint{
    context = ctx,
    name    = "z_angular",
    expr    = coord_z(getRotVec(rotation(tf))) - desired_omega_z*time,
    K       = 0,
    weight  = 1,
    priority= 2
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
