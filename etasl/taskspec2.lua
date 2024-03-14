require("context")

-- get input:
inp    = ctx:createInputChannelScalar("sine_input")

-- parameters for control :
des_x = inp
-- des_y = 0.2

-- variable definitions :
joint1 = Variable{context=ctx, name="joint1",vartype="robot"}
joint2 = Variable{context=ctx, name="joint2",vartype="robot"}

-- f = Variable{context=ctx, name="f",vartype="feature"}

-- -- definition of kinematic chain:
-- L = 0.2
-- ee_x = (L+f)*cos(joint1)
-- ee_y = (L+f)*sin(joint1)

-- definition of constraints:
Constraint{
    name    = "tracking_1",
    context = ctx,
    expr    = joint1,
    target = des_x,
    K = 10
}

Constraint{
    name    = "tracking_2",
    context = ctx,
    expr    = joint2,
    target = 0.05,
    K = 10
}



ctx:setOutputExpression("time",time)
ctx:setOutputExpression("joint1",joint1)
ctx:setOutputExpression("joint2",joint2)
ctx:setOutputExpression("inp",inp)

-- ctx:setOutputExpression("f",f)
-- ctx:setOutputExpression("dx",ee_x-des_x)
-- ctx:setOutputExpression("dy",ee_y-des_y)

-- Monitor {
--     context = ctx,
--     name    = "time_elapsed",
--     expr    = time,
--     upper   = 10.0,
--     actionname = "exit"
-- }

Monitor {
    context = ctx,
    name    = "time_elapsed",
    expr    = time,
    upper   = 5.0,
    actionname = "print",
    argument = "addtional argument"
}


