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
    worldmodel="../worldmodels/ur10_robot.json"
});
worldmodel = parameters["worldmodel"]


--
-- Reading world model :
-- needs to be loaded such that the default tasks knows about the available joints that it needs
-- to publish on a topic
--
WM = require("worldmodel")
URDF = require("urdfreader")
variable_list = {}
wm = WM.WorldModel.read_json(worldmodel)
robot_expressions = wm:getExpressions(variable_list, ctx, { { 'tool0', 'world' }, { "wp", "world" } })
ee = robot_expressions[1]
wp = robot_expressions[2]

