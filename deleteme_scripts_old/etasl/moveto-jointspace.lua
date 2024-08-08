require("context")
geom = require("geometric")
prm = require("parameters")
require("pretty")
local p = Pretty:new()

--
-- Reading parameters :
-- Specifying some defaults used when calling stand-alone,
-- and converting everything to radius if specified otherwise
parameters = prm.get_parameters({ 
    jointvalues = {
        shoulder_pan_joint = 0,
        shoulder_lift_joint =  -90,
        elbow_joint =  90,
        wrist_1_joint =  -90,
        wrist_2_joint = -90,
        wrist_3_joint = 0
    },
    units = "deg",
    maxvel = 0.5,
    maxacc =  0.5,
    worldmodel="../worldmodels/ur10_robot.json"
});

units = parameters["units"]
if units=="deg" then
    units = pi/180.0
elseif units=="rad" then
    units = 1
else
    error("units parameter should be 'deg' or 'rad' ")
end

jointvalues = parameters["jointvalues"]
for k,v in pairs(jointvalues) do
    jointvalues[k] = jointvalues[k]*units
end

maxvel = parameters['maxvel']*units
maxacc = parameters['maxacc']*units
worldmodel = parameters["worldmodel"]


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


mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
for k,v in pairs(jointvalues) do
    mp:addOutput(initial_value(time-0.01, ctx:getScalarExpr(k)), constant(v), constant(maxvel), constant(maxacc) )
end
cnt = 0
for k,v in pairs(jointvalues) do
    Constraint {
        context = ctx,
        name = "jointcnstr_"..k,
        expr = ctx:getScalarExpr(k),
        target = get_output_profile(mp,cnt),
        K = 4,
        weight = 1,
        priority = 2
    }
    cnt = cnt + 1
end

Monitor{
    context = ctx,
    expr = mp:get_duration() - time,
    lower = 0,
    actionname = "exit"
}



--- mp = create_motionprofile_trapezoidal()
--- mp:setProgress(time)
--- maxvel = constant(0.5)
--- maxacc = constant(0.5)
--- mp:addOutput(constant(0), constant(1.0), maxvel, maxacc)
--- mp:addOutput(constant(-1), constant(2.0), maxvel, maxacc)
--- duration = get_duration(mp)
--- x = get_output_profile(mp,0)
--- y = get_output_profile(mp,1)