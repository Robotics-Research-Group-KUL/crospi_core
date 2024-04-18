-- a four-bar mechanism.
-- 
-- q1 is the driving joint, the others are passive
--
--             q2 ---L23--- c
--             |            \
--             L12          L34
--             |              \
--             q1 --- L14----- q4
--             
--
--             ^ y
--             |
--             o --> x
--
-- (c = location of position constraint)
--
-- E. Aertbelien (c) 2022

require "context"
geom = require "geometric"
prm = require("parameters")


parameters = prm.get_parameters({amplitude=0.2,frequency=0.5,function_generator=false})
amplitude = parameters['amplitude']
frequency = parameters['frequency']
function_generator = parameters["function_generator"]



if function_generator then
    inp =amplitude*sin(2*pi*time*frequency) -- internally generated input
else
    inp = ctx:createInputChannelScalar("sine_input")
end

q1 = Variable{context=ctx,name='q1',vartype='robot'}
q2 = Variable{context=ctx,name='q2',vartype='feature',initial=0.1}
q4 = Variable{context=ctx,name='q4',vartype='feature',initial=0.2}

L12 = 0.5
L23 = 1.2
L34 = 1
L14 = 1

chain1 = geom.rotate_z(q1)*geom.translate_y(L12)*geom.rotate_z(q2)*geom.translate_x(L23)

chain2 = geom.translate_x(L14)*geom.rotate_z(q4)*geom.translate_y(L34)

closure = origin(chain1)-origin(chain2)

-- driving joint q1:
Constraint{
    context = ctx,
    expr    = q1 - inp, --0.5*sin(2*pi*time*0.1),
    K       = 4,
    priority= 1
};


Constraint{
    context = ctx,
    name    = 'loop-closure-x',
    expr    = coord_x(closure),
    K       = 4,
    priority = 1
};

Constraint{
    context = ctx,
    name    = 'loop-closure-y',
    expr    = coord_y(closure),
    K       = 4,
    priority = 1
};

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("q1",q1)
ctx:setOutputExpression("q2",q2)
ctx:setOutputExpression("q4",q4)
ctx:setOutputExpression("inp",inp)

ctx:setOutputExpression("closure_x",coord_x(closure))
ctx:setOutputExpression("closure_y",coord_y(closure))


Monitor {
    context = ctx,
    name    = "time_elapsed",
    expr    = time,
    upper   = 20.0,
    actionname = "exit"
}

