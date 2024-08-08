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

--
-- task specification
--

-- ========================================= PARAMETERS ===================================
maxvel = constant(0.1)
maxacc = constant(0.1)

print("========================------------==============")
-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value

-- end_j = { 240/180*math.pi, -110/180*math.pi, 110/180*math.pi, -90/180*math.pi, -90/180*math.pi, 45/180*math.pi }
-- end_j = { 30/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi, 0/180*math.pi,0/180*math.pi }
end_j = { 31/180*math.pi, 72/180*math.pi, -25/180*math.pi, -77/180*math.pi, -25/180*math.pi, 50/180*math.pi,  43/180*math.pi }


-- end_j = {0.506146666666667,0.57596,-0.750493333333333,-1.48353333333333,-0.610866666666667,1.11701333333333,-0.174533333333333}

for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), constant(end_j[i]), maxvel, maxacc)
end




-- The following creates a trapezoidal velocity profile from the initial value of each angle, towards the target angle. It checks whether the joint is continuous or bounded,
-- and if it is continuous it takes the shortest path towards the angle. This makes the skill generic to any type of robot (e.g. the Kinova).
-- The old version used the above commented method, which is the one that is explained in the etasl tutorial.
-- print("kasdjaksjdhkahskdhajkshdjkahsjkhdkjahsdhka")
-- for i=1,#robot_joints do
--     current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
--     local theta_init = initial_value(time, current_jnt[i])
--     local theta_final_raw = end_j[i]
--     print(theta_final_raw)
--     local difference_theta = cached(acos(cos(theta_init)*cos(theta_final_raw)+sin(theta_init)*sin(theta_final_raw))) --Shortest angle between two unit vectors (basic formula: 'cos(alpha) = dot(a,b)'. where a and b are two unit vectors)
--     local error_difference_theta = cached(acos(cos(theta_init + difference_theta)*cos(theta_final_raw)+sin(theta_init + difference_theta)*sin(theta_final_raw))) --Shortest angle computation also. If the sign is correct, it should be zero
--     local delta_theta = cached(conditional(error_difference_theta - constant(1e-5) ,constant(-1)*difference_theta,difference_theta)) --determines the proper sign to rotate the initial angle

--     local is_continuous = ctx:createInputChannelScalar("continuous_j"..i,0)--TODO: In the next release we will be able to obtain this directly from the urdf
--     local final_angle = cached(conditional(constant(-1)*abs(is_continuous),theta_final_raw,theta_init + delta_theta)) -- Only 0 is interpreted as bounded
--     mp:addOutput( theta_init, make_constant(final_angle) , maxvel, maxacc)
-- end

duration = get_duration(mp)
-- print(duration:value())

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory"..i,
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=4
    };

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - initial_value(time, current_jnt[i]),
    --     priority = 2,
    --     K=1
    -- };


    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - end_j[i] ,
    --     priority = 2,
    --     K=1
    -- };

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i]*time ,
    --     priority = 2,
    --     K=0
    -- };

    
end

--    Constraint{
--         context=ctx,
--         name="joint_trajectory6",
--         expr= current_jnt[6] - 20*3.1416/180 ,
--         priority = 2,
--         K=1
--     };

-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-duration +constant(0.2)
}

-- Monitor {
--     context = ctx,
--     name    = "time_elapsed",
--     expr    = time,
--     upper   = 1.0,
--     actionname = "print",
--     argument = "addtional argument"
-- }




ctx:setOutputExpression("time",time)

for i=1,#robot_joints do
    ctx:setOutputExpression("jpos"..i,current_jnt[i])
end
