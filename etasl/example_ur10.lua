require("context")
worldmodel=require("worldmodel")
urdfreader=require("urdfreader")

--
-- read robot model:
--
xmlstr = urdfreader.loadFile("/workspaces/colcon_ws/src/etasl_ros2/robot_description/urdf/ur10/use_case_setup_ur10.urdf")
robot = urdfreader.readUrdf(xmlstr,{})
robot:writeDot("ur10_robot.dot")
VL = {}
rv = robot:getExpressions(VL,ctx,{ee={'tool0','base_link'}})
ee = rv['ee']
--
-- task specification
--

inp    = ctx:createInputChannelScalar("sine_input")

pose_init = initial_value(time, ee)


sine_wave = coord_x(origin(pose_init ))+ inp

r_ee = rotation(ee)
r_ee_init=initial_value(time,r_ee)	

-- tgt_x = ctx:createInputChannelScalar("tgt_x",0.7)
-- tgt_y = ctx:createInputChannelScalar("tgt_y",0)
-- tgt_z = ctx:createInputChannelScalar("tgt_z",0.7)

Constraint{
    context=ctx,
    name="x",
    expr = coord_x(origin(pose_init )) - coord_x(origin(ee)),
    priority = 2,
    K        = 4
}
Constraint{
    context=ctx,
    name="y",
    expr = coord_y(origin(pose_init )) + sine_wave - coord_y(origin(ee)),
    priority = 2,
    K        = 4
}
Constraint{
    context=ctx,
    name="z",
    expr = coord_z(origin(pose_init )) - coord_z(origin(ee)),
    priority = 2,
    K        = 4
}

Constraint {
    context         = ctx,
    name            = "keep_rot",
    expr            = r_ee*inv(r_ee_init),
    weight          = 1,
    priority        = 2,
    K               = 4
};



ctx:setOutputExpression("time",time)
ctx:setOutputExpression("inp",inp)
