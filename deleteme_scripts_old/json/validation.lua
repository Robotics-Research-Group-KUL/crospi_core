JSON = require("JSON")
require("io")
require("pretty")
p = Pretty:new()
require("context")

local function is_array(table)
    if type(table) ~= 'table' then
      return false
    end
    if #table > 0 then
      return true
    end
    for k, v in pairs(table) do
      return false
    end
    return true
end

local function readjson(filename)
    local f = io.open(filename)
    if not f then
        error("could not open file '"..filename.."'")
    end
    local s=f:read("*all")
    f:close()
    --print(s)
    local retval = JSON:decode(s)
    -- p(retval)
    return retval
end


function check( cond, ctx, msg)
    local errormsg
    if cond then
        if ctx~= nil then
            errormsg = ctx.." : "..msg;
        else
            errormsg = msg;
        end
        error(errormsg,2)
    end
end





function add_schema(references, filename)
    if references["$defs"]==nil then
        references["$defs"] = {}
    end
    local f = io.open(filename)
    if not f then
        error("could not open file '"..filename.."'")
    end
    local s=f:read("*all")
    f:close()
    local retval = JSON:decode(s)
    if retval~=nil then
        check(retval["$id"] == nil, filename, "schema does not have an $id")
        references["$defs"][ retval["$id"] ] = retval
    else
        check(true,filename,"schema is nil")
    end
end


function transform_pre(schema, data, transform)
    local retval=data
    if transform=="from_etasl" then 
        if extendedtype(data) == "Vector" then
            retval={}
            retval["x"] = data:x()
            retval["y"] = data:y()
            retval["z"] = data:z()
        elseif extendedtype(data) == "Quaternion" then
            retval={}
            retval["w"] = data:w()
            retval["x"] = data:vec():x()
            retval["y"] = data:vec():y()
            retval["z"] = data:vec():z()
        elseif extendedtype(data) == "Rotation" then
            local q = toQuat(data)
            retval={}
            retval["w"] = q:w()
            retval["x"] = q:vec():x()
            retval["y"] = q:vec():y()
            retval["z"] = q:vec():z()            
        elseif extendedtype(data) == "Frame" then
            retval={}
            retval["origin"]      = transform_pre( schema.properties["origin"], data:Origin(), transform)
            retval["orientation"] = transform_pre( schema.properties["orientation"], data:Rotation(), transform)
        elseif extendedtype(data) == "Twist" then --TODO: Santiago, check if this works
            retval={}
            retval["linear"]      = transform_pre( schema.properties["linear"], data:Transvel(), transform)
            retval["angular"] = transform_pre( schema.properties["angular"], data:Rotvel(), transform)
        end
    end
    return retval
end


function transform_post(schema, data, transform)
    local retval=data
    if transform=="to_etasl" then 
        if schema.transform_to == "vector" then
            retval = Vector(data["x"], data["y"], data["z"])
        elseif schema.transform_to == "quaternion" then
            retval  = Quaternion(data["w"], data["x"], data["y"], data["z"])
            local n = norm(retval)
            check(math.abs(n - 1.0) > 1E-3, ctx, "quaternion is not normalized")
            retval = retval / norm(retval)
        elseif schema.transform_to=="frame" then
            local R = toRot(data["orientation"])
            local p = data["origin"]
            return Frame(R,p)
        elseif schema.transform_to=="twist" then
            local linear = data["linear"]
            local angular = data["angular"]
            retval = Twist(linear,angular)
        else
            check(schema.transform_to~=nil,ctx,"do not now how to handle transform_to value")
        end
    end
    return retval
end


---
--- validate( ctx, schema, data)
---
--- Validate the lua table data (from JSON) using the lua table schema containing a JSON shema
--- 
--- @param ctx     string indicating the context to help user situate the error
--- @param schema  table containing schema
--- @param data    table containing data
--- @param transform  string "to_etasl" or "from_etasl" or "": transform to contents to/froms eTaSL values 
--- @param references table containing $ref references to additional schema
--- @return nil    
---
--- catching error in lua:
---   local retval, err = pcall( validate("root", schema, data) )
---   if not retval then
---     print(err)
---   end
function validate(ctx, schema,  data, transform, references)
    if transform == nil then
        transform = ""
    end
    local retval = data;
    check(schema == nil, ctx, " schema should not be zero")
    if schema["$ref"] ~= nil then
        if references~= nil and references["$defs"] ~= nil then
            schema = references["$defs"][schema["$ref"]]
            check(schema == nil, ctx, "could not find reference $ref")
        else
            check(true, ctx, "could not find reference $ref (empty references table given)")
        end
    end
    check(schema.type == nil, ctx, "schema should always have member 'type'")
    data=transform_pre(schema,data,transform)
    if schema.type == "object" then
        check(not (type(data) == "table" and not is_array(data)), ctx, "should be an object according to schema")
        if schema.required ~= nil then
            check(type(schema.required) ~= "table", ctx, "'required' member of schema should be an array of strings")
            for _, prop in pairs(schema['required']) do
                check(type(prop) ~= "string", ctx, "required member of schema should be an array of strings")
                check(data[prop] == nil, ctx, "required member '" .. prop .. "' missing")
            end
        end
        check(schema.properties == nil, ctx, "schema of type object should have a properties member")
        retval = {}
        local size = 0
        for prop, val in pairs(data) do
            size = size + 1
            --check(schema.properties[prop] == nil, ctx, "contains unknown member '" .. prop .. "'")
            if schema.properties[prop] ~= nil then
                retval[prop] = validate(ctx .. "." .. prop, schema.properties[prop], val, transform, references)
            else
                if schema.additionalProperties ~= nil then
                    if schema.additionalProperties == false then
                        check(true, ctx,
                            " unknown property '" ..
                            prop .. "' encountered but schema does not allow additional properties")
                    else
                        retval[prop] = validate(ctx .. "." .. prop, schema.additionalProperties, val, transform,
                            references)
                    end
                end
            end
        end
        for prop, val in pairs(schema.properties) do
            print("checking "..prop)
            if (val.default ~= nil) then
                -- check correctness of this part !!!!
                print('default ' ..val.default)
                --- endand retval[prop] == nil then
                size = size+1
                retval[prop] = val.default
            end
        end
        if schema.maxProperties ~= nil then
            check(type(schema.maxProperties) ~= "number", ctx, "maxProperties member in schema should be a number")
            check(size > schema.maxProperties, ctx,
                " number of properties is larger than maxProperties (" .. schema.maxPropertes .. ")")
        end
        if schema.minProperties ~= nil then
            check(type(schema.minProperties) ~= "number", ctx, "minProperties member in schema should be a number")
            check(size < schema.minProperties, ctx,
                " number of properties is larger than minProperties (" .. schema.maxPropertes .. ")")
        end
        retval = transform_post(schema,retval,transform)
    elseif schema.type == "string" then
        check(type(data) ~= "string", ctx, "should be a string")
    elseif schema.type == "number" then
        check(type(data) ~= "number", ctx, "should be a number")
        if schema.minimum ~= nil then
            check(type(schema.minimum) ~= "number", ctx, " minimum value in schema should be number")
            check(data < schema.minimum, ctx, "number should be larger than minimum " .. schema.minimum)
        end
        if schema.maximum ~= nil then
            check( type(schema.maximum)~="number",ctx," minimum value in schema should be number")
            check( data > schema.maximum, ctx, "number should be smaller than maximum "..schema.maximum)
        end
    elseif schema.type == "boolean" then
        check(type(data) ~= "boolean", ctx, "should be a boolean")
    elseif schema.type == "array" then
        check(not (type(data) == "table" and is_array(data)), ctx, "should be an array according to schema")
        if schema.minItems ~= nil then
            check(type(schema.minItems) ~= "number", ctx, "minItems property of schema should be a number: ")
            check(#data < schema.minItems, ctx, "array should have at least " .. schema.minItems .. " elementss")
        end
        if schema.maxItems ~= nil then
            check(type(schema.maxItems) ~= "number", ctx, "maxItems property of schema should be a number")
            check(#data > schema.maxItems, ctx, "array should have at most " .. schema.maxItems .. " elements")
        end
        local itemschema = schema["items"]
        retval = {}
        local size = 0
        for i, val in ipairs(data) do
            size = size + 1
            if schema["items"] ~= nil then
                retval[i] = validate(ctx .. "[" .. i .. "]", schema["items"], val, transform, references)
            end
        end
        retval = transform_post(schema, retval,transform)
    else
        check(true, ctx, "type should be object, array,string or number")
    end
    return retval
end





function bundled_schema( schema, references)
    schema["$defs"] = references["$defs"]
    return JSON:encode_pretty(schema)
end


-- calling and catching validation errors:
-- local retval, err = pcall( validate, "root", schema, data )
-- if not retval then
--     print(err)
-- end

-- calling and raising an error to the caller of lua:


references = {}
add_schema(references, "vector.json")
add_schema(references, "twist.json")
add_schema(references, "orientation.json")
add_schema(references, "frame.json")
add_schema(references, "moveto.json")


data = readjson("value3.json")

-- p(references)
data2 = validate("root", references["$defs"]["moveto-schema.json"], data, "to_etasl", references)

--p(data2)

data3 = validate("root", references["$defs"]["moveto-schema.json"], data2, "from_etasl",references)

-- print( bundled_schema( { ["$ref"] = "moveto.json" } ,references))

p(data2)