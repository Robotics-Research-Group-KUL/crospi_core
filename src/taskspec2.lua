require("context")

-- get input:
inp    = ctx:createInputChannelScalar("sine_input")

-- parameters for control :
des_x = inp
-- des_y = 0.2

-- variable definitions :
q = Variable{context=ctx, name="q",vartype="robot"}
-- f = Variable{context=ctx, name="f",vartype="feature"}

-- -- definition of kinematic chain:
-- L = 0.2
-- ee_x = (L+f)*cos(q)
-- ee_y = (L+f)*sin(q)

-- definition of constraints:
Constraint{
    name    = "tracking_x",
    context = ctx,
    expr    = q,
    target = des_x,
    K = 10
}


ctx:setOutputExpression("time",time)
ctx:setOutputExpression("q",q)
ctx:setOutputExpression("inp",inp)

-- ctx:setOutputExpression("f",f)
-- ctx:setOutputExpression("dx",ee_x-des_x)
-- ctx:setOutputExpression("dy",ee_y-des_y)

Monitor {
    context = ctx,
    name    = "time_elapsed",
    expr    = time,
    upper   = 10.0,
    actionname = "exit"
}

Monitor {
    context = ctx,
    name    = "time_elapsed",
    expr    = time,
    upper   = 5.0,
    actionname = "print",
    argument = "addtional argument"
}


