---@meta

-- should be in the original library files, but for now in a separate file

-- https://github.com/LuaLS/lua-language-server/wiki/Formatting-Annotations
-- https://github.com/LuaLS/lua-language-server/wiki/Annotations#class


---@class Vector
Vector = {}
--@class Frame
Frame = {}
---@class expression_double
expression_double = {}

---@class Context
ctx = {}

---set and output expression 
---@param name name of the output
---@param expr expression that will be evaluated and send to the output
function ctx:setOutputExpression(name,expr) end




--- creates a scalar input channel 
--- @param name string name of the input channel
--- @param defaultvalue number default value for the input channel (when no input has arrived and no previous input is available)
--- @return integer index index for this variable (typically ignored)
function ctx:createInputChannelScalar(name, defaultvalue) end

---get the name of the scalar variable with a given index
---@param ctx context 
---@param i integer number indicating the index.
function ctx:getScalarName(ctx, i) end



--- Gets an expression corresponding to the variable with the given name
--- @param name string name of a variable
--- @return  expresssion expression for the given variable
function ctx:getScalarExpr(name) end


--- get the type of a scalar variable given its index
---@param ndx integer index of the scalar variable
---@return string string describing the type ("time","robot" or "feature")
function ctx:getScalarType(ndx) end

---gets the weight of a scalar variable specified by its index
---@param ndx integer index of the scalar
---@return number number the weight of the scalar variable
function ctx:getScalarWeight(ndx) end

---gets the initial value fo a scalar variable specified given by its index
---@param ndx integer index of the scalar variable
function ctx:getScalarInitialValue(ndx) end

---gets the index corresponding to the scalar variable with the given name
---@param name string name of the scalar variable
---@return integer index index of the scalar variable
function ctx:getScalarNdx(name) end


-- Compute the union of namedvalues
-- @param a number[string]
function union(a,b) end

--  **Constraint** (...) specifies constraints in a way  that takes into account
--  changes of the weight variables in its feedforward.<br>
--  This function is called as follows ( not all combinations of arguments are possible ):<br>
--  **Constraint{**
--   -  context = ... 
--   -  name = ...         [optional, default_name<nr>]
--   -  model = ...  (expression)
--   -  meas = ...   (expression)
--   -  expr = ...[ compatibility, if expr is used then model==expr and meas==expr ]
--   -  target = ...       [optional, 0.0] ( can be expression )
--   -  target_lower = ... [optional, 0.0] ( can be expression )
--   -  target_upper = ... [optional, 0.0] ( can be expression )
--   -  K = ...            [optional, 4.0] ( the gain, i.e. inverse of the time constant of the first order
--   -                     linear system describing the behavior)
--   -  weight = ...       [optional, defaultweight, >= 0] ( can be expression )
--   -  priority = ...     [optional, defaultpriority, 0..2]
--   -  controller_lower = ....  [optional, 'proportional']
--   -  controller_upper = ....  [optional, 'proportional']
--   -  controller       = ....  [optional, 'proportional']
--   -  controllerparameter =... (optional, can be expressions)
--   -  controllerparameter_lower or controllerparameter_upper (optional, can be expressions) <br>
--  **}** <br>
--  The xxx_lower and xxx_upper variables can only be used when the type of expr is expression_double; they
--  can be scalars or scalar expressions.
--  If omitted context.defaultweight, context.defaultpriority, context.defaultK are used
--  For other variables, the most evident default is taken or an error message is given if this does not exists.<br>
--  
--  target_lower==-1E20 or target_upper==1E20 is used as a flag to indicate that this boundary is inactive, so
--  setting these specific values will still work when K==0!
-- 
function Constraint(arg) end




--  This function is called as follows :
--  Variable { 
--     context  = ... [the context to define the variable in, typically ctx]
--     name     = ... [string representing the name of the variable]
--     vartype  = ... [optional, 'robot' or 'feature', defaults  to 'feature']
--     initial  = ... [optional number, defaults to  0.0]
--     weight   = ... [optional scalar expression, defaults to constant(1.0)]
--  }
--
--  returns a scalar expression referring to the variable
--   
function Variable(arg) end


--   **BoxConstraint{**<BR>
--       context=...,   [context to use]<BR>
--       var_name=..,    [variable of which you want to constrain the velocity]<BR>
--       lower=..,       [optional, default -large_value][lower velocity bound]<BR>
--       upper=..,       [optional, default +large_value][upper velocity bound]<BR>
--       tags=..         [optional, default ''][tags for this constraint]<BR>
--   **}**<BR>
--<BR>
function BoxConstraint() end

-- Imposes **velocity** constraints on a scalar position-level expression ( not all combinations of arguments are possible ):<BR>
-- **VelocityConstraint**{
--  -  context         = ... 
--  -  name            = ...    [optional, default_name<nr>]
--  -  expr            = ... (! scalar position-level expression !)
--  -  target_velocity = ...       [optional, 0.0]
--  -  weight          = ...       [optional, defaultweight] (scalar)
--  -  priority        = ...     [optional, defaultpriority]
-- **}**
function VelocityConstraint(arg) end

--  deriv(e, varname)
--  
--  INPUT:
--      e:  an expression
--      varname: the name of a variable
--  OUTPUT:
--      the partial derivative of the expression towards the variable.
--  
function deriv( e, varname )

-- addJointsToOutput(u,ctx,typename) adds all joints of a robot as an output\
--   u is an UrdfExpr object, ctx is a Context object and typename is the handler name
--   for the output
--  
function addJointsToOutput(u,ctx, typename) end

-- uses roslines to output a frame representation to ROS.
-- X: red, Y: blue, Z: cyan
--
function outputFrame(ctx,name,F)

--  addJointsToOutput(u,ctx) adds all joints of a robot as an output expression\n"..
--   u is an UrdfExpr object, ctx is a Context object \n"..
--   for the output\n\n";
function addJointsToOutputExpression(u,ctx) end

-- Defines a monitor that checks a condition and generates an event:<BR>
-- **Monitor {**
--  -  context = the context to which to add the monitor, typically *ctx*
--  -  name    = [optional, default_name<nr>] An optional name for the monitor
--  -  expr    = scalar expression to monitor.  The event is triggered when this
--  expression **exceeds** its lower bound or upper bound.
--  -  lower   = ... [optional, default -infinity] indicates the lower bound for the expression
--  -  upper   = ... [optional, default +infinity] indicates the upper bound for the expression
--  -  actionname    = name of the event. The "**exit**" event is always defined.
--  -  argument= ...[optional, ''] <BR>
-- **}** <BR>
-- <BR>
-- **Warning**: lower and upper cannot be both unspecified.
--  
function Monitor(arg) end


-- returns an expression for the derivative towards variable with index *ndx*.
function derivative(ndx) end



---Makes an expression instantaneously constant, i.e. ensures that the derivative towards all
---variables is zero.  With instantaneeously, we mean that the **value** of the expression can
---still change over time, but that its derivatives are zero.
---
---This is a concept that occurs often in robotics:  for example,
---the Jacobian of a robot is often expressed with respect to an instantaneously constant frame
---coinciding with the end effector.  If we would directly express the forward position kinematics of the end effector
---with respect to the end effector and use automatic differentiation to compute the Jacobian,  this Jacobian result would always be zero. 
---With the ideom `inv(make_constant(ee_wrt_base)) * ee_wrt_base`, we can use an expression `ee_wrt_base` 
---for the end effector with respect to the base and express it in an instantaneously constant coinciding 
---frame with the end effector using `inv(make_constant(ee_wrt_base))`.
---
---Implementation of this function is very simple.  It zeroes out all derivatives of a given 
---expression.
---
---@param arg expression  expression to make instantaneously constant (can be of any type)
---@result expression e expression with derivative towards all variables zero, independent of the derivatives of the argument.
function make_constant(arg) end


-- Returns an expression that is equal to the expression of its input at time 0:
--  - *time* time variable
--  - *arg* expression that can (of course) changes over time.
-- 
-- Returns an expression of a constant value corresponding to the value of *arg* at time 0. For all
-- values of *time* before *0*, the returned expression is equal to *arg*.  It is assumed that *time*
-- increases monotoneously.  This is the case for the standard eTaSL solver.
function initial_value(time, arg) end


-- Generates a GraphViz .dot file of the expressions in *exprs* and writes it to the file
-- with name *filename*:
--  - *filename*: filename of the file to write to
--  - *exprs*: a LUA table of expressions
-- 
-- The .dot file can be viewed by e.g. xdot or a pdf can be generated from this file using the dot-tool.
function write_expressions_to_dot( filename, exprs)



--  Defines a joint or feature variable in the given context
--  **Variable { ** <BR>
--   -  context  = ...
--   -  name     = ...
--   -  vartype  = ... [optional, default 'feature']
--   -  initial  = ... [optional, default 0.0]
--   -  weight   = ... [optional, default 1.0]<BR>
--  **}**<BR>
---@returns expression_double an expression graph pointing to the variable
function Variable(arg) end


-- returns an expression pointing to a cache of the argument:
--  - *arg* : expression to cache
-- 
-- Returns:
--  - an expression that will only be evaluated once, even when it is used multiple times.
--
function cached(arg) end     


-- returns an expression of a constant value
function constant(arg) end

------------------------------------------------------------------------------------------------------
-- Quaternion functions (only development version of eTaSL !!!)
------------------------------------------------------------------------------------------------------


-- creates a normalized version of the argument:
--  - q : a 3-vector or quaternion (expression or value)
--
-- Returns:
--  - normalized vector or quaternion.
function normalized(q) end

-- returns the conjugate of q: 
--  - *q* : quaternion (expression or value)
function conj(q) end

-- returns the real part:
--  - *q* : quaternion (expression or value)
function w(q) end

-- returns the vector part:
--  = *q* quaternion *q* (expression or value)
function vec(q) end

-- returns the squared norm of a:
--  - *a* : vector or quaternion (expression or value)
function squared_norm(a) end


-- returns the norm of a:
-- - *a* : vector or quaternion (value or expression)
function norm(a) end

-- returns the inverse of a:
-- - *a* : quaternion, rotation, frame (value or expression)
function inv(a) end

-- apply the transformation in a to b:
--  - *a* : unit vector, rotation or frame (value or expression)
--  - *b* : vector (value or expression)
function apply(a,b) end

-- The natural logarithm of a unit quaternion:
-- - *a* : a unit quaternion, i.e. with norm equal to 1 (value or expression)
--
-- Returns:
-- - a pure quaternion. i.e. a quaternion with its real part, w(..)  equal to zero.
function logUnit(a)   end

-- The power of a unit quaternion *a* to the order *b*:
-- - *a*: a unit quaternion, i.e. with norm equal to 1 (value or expression)
-- - *b*: a scalar number specifying the order (value or expression)
-- 
-- Returns:
--  - a unit quaternion
function powUnit(a,b)     end

-- The power of a quaternion *a* to the order *b*:
-- - *a*: a quaternion (value or expression)
-- - *b*: a scalar number specifying the order (value or expression)
-- 
-- Returns:
--  - a  quaternion
function pow(a,b)     end

-- The **geodesic distance** between two unit quaternions:
-- - *a*: a unit quaternion, i.e. with a norm equal to 1 (expression or value)
-- - *b*: a unit quaternion, i.e. with a norm equal to 1 (expression or value)
--
-- Returns:
--  - A pure quaternion (i.e. with real part equal to zero) representing the 
-- geodesic distance, i.e. the rotational velocity to go from a to b in 1 second.
--    
function diffUnit(a,b)        end

-- converts its arguments to a unit quaternion.  The following options are
-- available:
--  - toQuat(*a*), with the vector *a* representing the axis-angle representation, i.e.
-- the rotational velocity to obtain the orientation in 1 second (expression or value).
--  - toQuat(*axis*,*angle*), representing the axis angle representation with *axis* the
--  direction and *angle* the angle about which the rotate (expression or value).
--  - toQuat(*R*), with *R* a rotation matrix (expression or value).
--
-- Returns:
--  - a unit quaternion representing the orientation.
function toQuat(a,b)    end

-- converts a quaternion to an axis-angle representation:
--  - *q* : a unit quaternion, i.e. with normal equal to one (expression or value).
-- 
-- Returns:
--  - vector representing the orientation using a rotational velocity that generates the rotation 
-- when applied during 1 second.
function axisAngle(q)  end

-- converts a quaternion to a rotation matrix:
-- - *R* : an orientation represented by a rotation matrix (value or expression)
--
-- Returns:
-- - *q* : a unit quaternion representing the orientation.
function toRot(a) end

-- spherical linear interpolation between q1 and q2:
--  - *q1*: a quaternion at s==0 (expression or value)
--  - *q2*: a quaternion at s==1 (expression or value)
--  - *s* : scalar number, the requested quaternion is evaluated for s (expression or value)
function slerp(q1,q2,s) end

-- spherical linear interpolation between q1 and q2:
--  - *q1*: a unit quaternion representing the orientation at s==0 (expression or value)
--  - *q2*: a unit quaternion representing the orientation at s==1 (expression or value)
--  - *s* : scalar number, the requested orientation is evaluated for s (expression or value)
function slerpUnit(q1,q2,s) end







------------------------------------------------------------------------------------------------------
-- Scalar functions:
------------------------------------------------------------------------------------------------------

-- the square of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the square of *a*
function sqr(a) end

-- the square root of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the square root of *a*
function sqrt(a) end

-- the sine of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the sine of *a*
function sin(a) end

-- the cosine of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the cosine of *a*
function cos(a) end

-- the tangent of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the tangemt of *a*
function tan(a) end

-- the arcsine of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the sine of *a*
function asin(a) end

-- the arccosine of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the cosine of *a*
function acos(a) end

--- the arctangent of a scalar value (can return values between -pi/2 ... pi/2)
--- @param a number scalar number (expression or value)
--- @return the tangemt of *a*
function atan(a) end

---arctangens of y/x.  Can returns values from -pi ... pi.
---@param y expression_double value for the y-coordinate
---@param x expression_double value for the x-coordinate
function atan2(y,x) end

-- the logarithm of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the logarithm of *a*
function log(a) end

-- the exponential of a scalar value:
--  -*a* : scalar number (expression or value)
-- 
-- Returns:
--  - the exponential of *a*
function exp(a) end



-- returns either *b* or *c* depending on the value of *a*:
--  - *a* if *a* >= 0, return *b*, otherwise return *c* (scalar expression)
--  - *b* expression to return (any type)
--  - *c* expression to return (any type)
function conditional(a, b, c) end

-- return the maximum of *a* and *b*:
--  - *a* : scalar value or expression
--  - *b* : scalar value or expression
function maximum(a, b) end

-- return the minimum of *a* and *b*:
--  - *a* : scalar value or expression
--  - *b* : scalar value or expression
function minimum(a,b) end

------------------------------------------------------------------------------------------------------
-- Vector, Rotations and Frames (VALUES)
------------------------------------------------------------------------------------------------------




-- create a vector expression with the given elements:
-- - *a*: scalar number (expression or value)
-- - *b*: scalar number (expression or value)
-- - *c*: scalar number (expression or value)
-- 
-- Returns:
--   a vector(a,b,c)
--
function vector(a,b,c) end


-- returns an *expression* for the **origin** of a frame expression
function origin(arg) end

-- returns an *expression* for the **orientation** of a frame expression
function rotation(arg) end

---constructs a frame expression from its arguments
---@param R expression_rotation rotation matrix (optional)
---@param v expression_vector vector (optional)
---@return expression_frame 
function frame(R,v) end

---constructs a twist expression
---@param v expression_vector translational velocity
---@param omega expression_vector rotational velocity
---@return expression_twist tw expression for the twist.
function twist(v,omega) end

---extracts the translational velocity from a twist expression.
---@param tw expression_twist expression for the twist.
---@return expression_vector v expression for the translational velocity.
function transvel(tw) end


---extracts the rotational velocity from a twist expression.
---@param tw expression_twist expression for the twist.
---@return expression_vector v expression for the rotational velocity.
function rotvel(tw) end


---constructs a wrench expression
---@param force expression_vector expression for the force
---@param torque expression_vector expression for the torque
---@return expression_twist tw expression for the wrench
function wrench(force,torque) end

---extracts the force from a wrench expression.
---@param wrench expression_wrench expression for the wrench.
---@return expression_vector force expression for the force
function force(wrench) end

---extracts the torque from a wrench expression.
---@param wrench expression_wrench expression for the wrench.
---@return expression_vector torque expression for the torque
function torque(wrench) end


-- gets the roll pitch yaw angles of an orientation:
--  - *R* : orientation matrix (expression)
--
-- Returns:
--  - *roll* around X (value)
--  - *pitch* around Y (value)
--  - *yaw*  around Z (value)
-- 
-- Two typical interpretations of these angles:
--  - First roll around a fixed X, pitch around a fixed Y and yaw around a fixed Z.
--  - yaw around Z, pitch around moving Y, roll around moving X.
function getRPY( arg ) end


---@class Rotation
Rotation={}

-- returns a rotation around the X-axis:
-- -*a*: rotation angle (value)
---@return Rotation
function Rotation.RotX(a) end

-- returns a rotation around the Y-axis:
-- -*a*: rotation angle  (value)
---@return Rotation
function Rotation.RotY(a) end

-- returns a rotation around the Z-axis:
-- -*a*: rotation angle  (value)
---@return Rotation
function Rotation.RotZ(a) end

-- returns a roll-pitch-yaw rotation:
--  - *roll* roll angle (around X)  (value)
--  - *pitch* pitch angle (around Y) (value)
--  - *yaw* yaw angle (around Z) (value)
---@return Rotation
function Rotation.RPY(roll,pitch,yaw) end


-- returns the Euler ZYX angles:
--  - *a* angle around Z (value)
--  - *b* angle around Y (value)
--  - *c* angle around X (value)
---@return Rotation
function Rotation.EulerZYX(a,b,c) end


-- returns the Euler ZYZ angles:
--  - *a* angle around Z (value)
--  - *b* angle around Y (value)
--  - *c* angle around Z (value)
---@return Rotation
function Rotation.EulerZYZ(a,b,c) end

-- returns identity rotation  (value)
---@return Rotation
function Rotation.Identity() end



-- member function that returns the inverse of a Rotation (value)
---@return Rotation
function Rotation:Inverse() end 


-- A class that encapsulates a trapezoidal motion profile.
---@class MotionProfile
MotionProfile={}


--- Creates a motion profile
---
--- a constructor that creates an object to manage trapezoidal motion profiles,
--- i.e. a velocity profile with a maximal velocity and maxium acceleration (+/-).
---
--- **EXAMPLE**:
--- ```
--- require("context")
--- mp = create_motionprofile_trapezoidal()
--- mp:setProgress(time)
--- maxvel = constant(0.5)
--- maxacc = constant(0.5)
--- mp:addOutput(constant(0), constant(1.0), maxvel, maxacc)
--- mp:addOutput(constant(-1), constant(2.0), maxvel, maxacc)
--- duration = get_duration(mp)
--- x = get_output_profile(mp,0)
--- y = get_output_profile(mp,1)
--- ```
--- <br>
--- <br>
---@return MotionProfile
function create_motionprofile_trapezoidal() end

-- gets an expression for the starting value for the idx-th output of this motion profile.
function MotionProfile:getStartValue(idx) end

-- gets an expression for the starting value for the idx-th output of this motion profile.
function MotionProfile:getEndValue(idx) end

-- sets the progress variable for this motion profile.
-- The motion profile is a function of this variable,
-- and the maximum acceleration and velocity is expressed
-- for this variable:
--  - *varexpr* : scalar expression for the progress variable.
function MotionProfile:setProgress(varexp) end


-- Defines an additional output for this motion profile:
--  - *startv* : double expression indicating the start value 
--  - *endv* : double expression indicating the end value 
--  - *maxv* : double expression indicating the maximum velocity 
--  - *max* : double expression indicating the maximum acceleration 
function MotionProfile:addOutput(startv,endv,maxv,maxa)



-- Gets the planned duration for a motion profile:
function MotionProfile:get_duration() end

--- gets an expresion for the `idx`-th output of the motion profile.
--- @param idx integer index of the output to give the expression back. 
--- @return expression_double e `idx`-th output of the motion profile.
function MotionProfile:get_output_profile( idx) end

--- gets an expresion for the `idx`-th output of the motion profile.
--- @param mp MotionProfile motion profiles
--- @param idx integer index of the output to give the expression back. 
--- @return expression_double e `idx`-th output of the motion profile.
function get_output_profile(mp, idx) end

-- gets an expression for the maximum velocity for the `idx`-th output.
function MotionProfile:getMaxVelocity(idx) end

-- gets an expression for the maximum acceleration for the *idx*-th output.
function MotionProfile:getMaxAcceleration(idx) end





-- removes dependencies, i.e. makes the specified derivatives zero for its argument:
--  - *ctx* context, used to lookup the names and associate them with indices.
--  - *arg* the expression of which you want to remove the dependencies.  Can be an expression
--  in function of any type.
--  - *names* names of the variables.  The dependencies on these variables will be removed, i.e.
--  the derivative of the expression towards these variables will be zero.
--
-- Returns:
--  - expresion *arg* with the dependencies removed.
function remove_dependencies(ctx,arg,names) end

-- checks whether the expression arg is non-null. Mostly for error checking with proper reporting.
--
function is_non_null(arg) end



-- returns the x-component of the vector.
function Vector.x() end

-- returns the x-component of the vector.
function Vector.y() end

-- returns the x-component of the vector.
function Vector.z() end

-- returns a zero vector
function Vector.Zero() end


-

-- gets the inverse of a frame
function Frame:Inverse() end

-- gets the origin of a frame
function Frame:Origin() end

-- gets the orientation part of a frame
function Frame:Rotation() end

-- returns an identity frame
function Frame.Identity() end



-- returns the current value of this scalar expression. 
function expression_double:value() end


---cross product between two expression_vectors
---@param a expression_vector first vector expression
---@param b expression_vector second vector expression
---@return expression_vector r   a x b
function cross(a,b) end


---returns dot product (i.e. inner product) of two vector expressions.
---@param a expression_vector first vector expression
---@param b expression_vector second vector expression
---@return number r   inner product of a with b
function dot(a,b) end


---returns expressions such that if e is near zero, a is returned and otherwise b.
---@param e expression_double expression to evaluate.
---@param tol number tollerance
---@param a expression_double alternative when near zero
---@param b expression_double alternative when **not** near zero
function near_zero(e, tol, a, b) end

---Squared norm
---@param v expression_vector vector expression
---@return expression_double n expression_double representing the squared norm. 
function squared_norm(v) end


---Norm
---@param v expression_vector vector expression
---@return expression_double n expression_dobule representing the norm of the vector expression argument
function norm(v) end


---x coordinate of a expression_vector
---@param v expression_vector
---@return expression_double x x-coordinate of v.
function coord_x(v) end

---y coordinate of a expression_vector
---@param v expression_vector
---@return expression_double y y-coordinate of v.
function coord_y(v) end

---z coordinate of a expression_vector
---@param v expression_vector
---@return expression_double z z-coordinate of v.
function coord_z(v) end


---rotation around a **fixed** axis
---@param v_x number x-value of the vector representing the axis.
---@param v_y number y-value of the vector representing the axis.
---@param v_z number z-value of the vector representing the axis.
---@param angle expression_double expression for the angle to rotate.
---@return expression_rotation r expression for the rotation.
function rot(v_x,v_y,v_z, angle) end


---returns rotation expression for a rotation around x
---@param angle expression_double rotation angle
---@return expression_rotation r expression for the resulting rotation matrix.
function rot_x(angle) end


---returns rotation expression for a rotation around y
---@param angle expression_double rotation angle
---@return expression_rotation r expression for the resulting rotation matrix.
function rot_y(angle) end


---returns rotation expression for a rotation around z
---@param angle expression_double rotation angle
---@return expression_rotation r expression for the resulting rotation matrix.
function rot_z(angle) end


---Gets the displacement rotation vector for a given rotation expression, i.e. 
---the rotational velocity such that if one rotates for one unit of time around this
---vector, one achieves the given rotation.
---
---**The derivatives computed by getRotVec are only correct around the identity matrix (i.e.**
---**resulting value is a zero vector).  This works as expected for a constraint that has**
---**expr==getRotVec(...) and target==Vector:Zero().  It is NOT correct for other targets.**
---
---The reason for this is that for non-identity matrices this derivatives is difficult to compute
--- 
---@param r expression_rotation expression for a rotation matrix.
---@return expression_vector v expression for the displacement rotation.
function getRotVec( r ) end


---returns an expression for the rotation matrix corresponding to a rotation around v with the given angle.
---In contrast to rot(x,y,z, angle) which specifies a constant axis, this routine can deal with a varying rotation axis.
---
---**The derivatives computed by rotVec are only correct around the zero angle (i.e.**
---**resulting value is an identity matrix). **
---
--- However, the following is still valid: getRotVec(rotVec(a/norm(a),a)) == a (if norm(a) is in [0..pi) )
---
---@param v expression_vector expression for the rotation axis (can be moving). The vector `v` should be ***normalized***.
---@param angle expression_double expression for the rotation angle.
---@return Rotation r ration corresponding to a rotation around v with the given angle.
function rotVec( v, angle) end

---returns a vector expression for the first column of the given expression for a rotation matrix
---@param r expression_rotation expression for the rotation matrix.
---@return expression_vector u1 first colulmn of the rotation matrix.
function unit_x(r) end

---returns a vector expression for the second column of the given expression for a rotation matrix
---@param r expression_rotation expression for the rotation matrix.
---@return expression_vector u2 second colulmn of the rotation matrix.
function unit_y(r) end

---returns a vector expression for the thirdcolumn of the given expression for a rotation matrix
---@param r expression_rotation expression for the rotation matrix.
---@return expression_vector u3 first colulmn of the rotation matrix.
function unit_z(r) end 


---Constructs a rotation matrix from expressions for three orthonormal vectors describing a right-handed frame.  Note that the vectors should be **normalized** and **perpendicular** to each other and should form a **right-handed** frame.
---@param u1 expression_vector first orthonormal vector
---@param u2 expression_vector second orthonormal vector
---@param u3 expression_vector third orthonormal vector
---@return expression_rotation r an expression for the rotation matrix.
function construct_rotation_from_vectors(u1,u2,u3) end


----------------------------------------------------------------------------------------------------------------------
-- Reading URDF's (old-style)
----------------------------------------------------------------------------------------------------------------------

---@class UrdfExpr
---A class that encapsulates the reading of an URDF-file.  Reading an URDF file uses the following steps:
--- - Constructing the reader object :  local u = UrdfExpr()
--- - Register the transformations you are interestd in :   u:addTransform(...)
--- - Get all the requested transformations and construct constraints related to the joint position and velocity limits specified in the URDF.  Only
---   the relevant parts of the URDF-file will be used.  If parts of the kinematic tree are not necessary to compute the requested transformations,
---   these parts (and their related constraints!) are never generated.  This is done using the u:getExpressions(ctx) method.
--- - Optionallly, the u:getAllJointNames() method returns all the joint names that are occurring in the expressions for the required transformations.  If the values of some
---   of the joints are not necessary to compute the requested transformations, these are joints are not included.
--- This object is created using an empty constructor:  u=UrdfExpr()
UrdfExpr = {}

---@return UrdfExpr
function UrdfExpr() end

---@method
---A method that reads an URDF from the file with the given filename.
---@param filename string file to read
function UrdfExpr:readFromFile(filename) end

---@method
---A method that registers a transform to an `UrdfExpr` object.  It will return the transformation of `link_of` with respect to `link_wrt`.
---@param name string getExpressions will return a table containing a field with this name containing an expression for the specified transformation.
---@param link_of string the name of a link in the URDF for which you want to express a transformation.
---@param link_wrt string the name of a link in the URDF with respect to which you want to express the transformation.
function UrdfExpr:addTransform(name, link_of, link_wrt) end

---@method
---A method that returns a table with transformations **AND** will generate all the related constraints to enforce the position and velocity limits specified in the URDF-file.
---The table will contain fields with the names specified by the method 'addTransform'.  These fields will contain expression_frame variables representing the specified transformations.
---@param ctx Context the eTaSL context in which the constraints will be specified.
---@return table r  a Lua table containing the requested transformations.
function UrdfExpr:getExpressions(ctx) end


---@method
---A method that returns all joint names that are involved in computing the requested transformations. (Not necessarily all of the joints in the URDF!)
---@return table jnts a lua table of strings indicating all the joints names.
function UrdfExpr:getAllJointNames() end
