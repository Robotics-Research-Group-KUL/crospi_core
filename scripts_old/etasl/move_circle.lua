require("context")
geom = require("geometric")
-- prm = require("parameters")
require("pretty")
local p = Pretty:new()

--
-- Reading parameters :
--
-- parameters = prm.get_parameters({ radius = 0.1, velocity = 0.1, worldmodel = '../worldmodels/ur10_robot.json' })

-- radius = parameters['radius']
-- velocity = parameters['velocity']
-- worldmodel = parameters["worldmodel"]

radius = 0.1
velocity = 0.1
worldmodel = '/home/santiregui/ros2_ws/src/etasl_ros2/scripts/worldmodels/ur10_robot.json'

omega = velocity / radius

--
-- Reading world model :
--
WM = require("worldmodel")
URDF = require("urdfreader")
variable_list = {}
wm = WM.WorldModel.read_json(worldmodel)
robot_expressions = wm:getExpressions(variable_list, ctx, { { 'tool0', 'world' }, { "wp", "world" } })
ee = robot_expressions[1]
wp = robot_expressions[2]


--
-- Task specification
--

target = wp * rotate_z(omega * time) * translate_x(radius) *rotate_y(pi)
target_orientation = wp*rotate_y(pi)

target_wrt_ee = cached(inv(ee) * target)
target_orientation_wrt_ee = cached( inv(ee)*target_orientation)

Constraint {
    context  = ctx,
    expr     = origin(target_wrt_ee),
    K        = 5,
    weight   = 1, -- weight for translational part
    priority = 2
}


Constraint {
    context  = ctx,
    expr     = rotation(target_orientation_wrt_ee),
    K        = 4,
    weight   = 0.1, -- weight for rotational part : 1 rad/s is equivalent to 10 cm/s
    priority = 2
}


Monitor {
    context = ctx,
    expr = time,
    upper = 4*pi,
    actionname = "exit"
}


Monitor {
    context = ctx,
    expr = time,
    upper = 4.0,
    actionname='print'
}

function setOutputExpressionPose(ctx, name, v)
    local q = cached(toQuat(rotation(target_wrt_ee)))
    local p = cached(origin(target_wrt_ee))
    ctx:setOutputExpression(name .. "_x", coord_x(p))
    ctx:setOutputExpression(name .. "_y", coord_y(p))
    ctx:setOutputExpression(name .. "_z", coord_z(p))
    -- ctx:setOutputExpression(name .. "_qr", real(q))
    ctx:setOutputExpression(name .. "_qx", coord_x(vec(q)))
    ctx:setOutputExpression(name .. "_qy", coord_y(vec(q)))
    ctx:setOutputExpression(name .. "_qz", coord_z(vec(q)))
end


ctx:setOutputExpression("time", time)
ctx:setOutputExpression("shoulder_pan_joint", variable_list.shoulder_pan_joint)
ctx:setOutputExpression("shoulder_lift_joint", variable_list.shoulder_lift_joint)
ctx:setOutputExpression("elbow_joint", variable_list.elbow_joint)
ctx:setOutputExpression("wrist_1_joint", variable_list.wrist_1_joint)
ctx:setOutputExpression("wrist_2_joint", variable_list.wrist_2_joint)
ctx:setOutputExpression("wrist_3_joint", variable_list.wrist_3_joint)
setOutputExpressionPose(ctx, "delta", target_wrt_ee)
setOutputExpressionPose(ctx,"target",target)
ctx:setOutputExpression("wp", wp);
