require("context")
require("geometric")

-- inspect = require 'inspect'

--- table to record schema info on parameter 
local vars_info_tab = {}

--- table to record variable names
local var_names_tab = {}

--- table to record wheter a parameter is optional
local var_optional_tab = {}

--- description of the task to be embeded in schema
local task_description = ""

-- maxvel = ctx:createInputChannelScalar("maxvel",0.5)

function set_task_description(task_descript)
    task_description = task_descript
end



---create a scalar parameter.
---
---@param var_name string parameter name
---@param default_val number default value of the parameter (not used for optional parameters)
---@param description_val string description of the parameter to be used in schema
---@param optional boolean whether or not the parameter is optional
---@return number|nil number if parameter is optional and not defined, nil is returned
---
function createScalarParameter(var_name, default_val, description_val,optional)
    -- local var_tab = {
    --         name=varname,
    --         default=default_val
    -- }
    -- table.insert(vars_info_tab,var_name)
    if not description_val then 
        description_val="The description was not defined. Please provide it when defining the input in the task specification lua file with the createScalarParameter function."
    end

    if optional==nil then
        optional=false
    else
        optional=true
    end
    var_optional_tab[var_name] = optional
    table.insert(var_names_tab, var_name)
    local description_suffix = ". Set as 'external' if the value is not yet known and thus will be set externally at runtime (only once) depending on e.g. the outcome of a previous action or the outcome of another module."
    if default_val then
        vars_info_tab[var_name] = {
            description = description_val .. description_suffix,
            default=default_val,
            oneOf= {{ type= "number" }, { enum= {"external"}}}
          }
    else
        vars_info_tab[var_name] = {
            description = description_val .. description_suffix,
            oneOf= {{ type= "number" }, { enum= {"external"}}}
          }
    end
    local exists = rawget(_G, var_name) ~= nil
    if exists then
        print(var_name .. " value: " .. rawget(_G, var_name))
        return rawget(_G, var_name)
    else
        if not optional then
            print("Warning: " .. var_name .. " does not exist and thus the default value is used. If this code was executed during generation of JSON Schemas, please ignore this warning.")
            return default_val
        else
            return nil
        end
    end
    -- return ctx:createInputChannelScalar(var_name, default_val)
end


---
---Create  a string parameter representing one of a list of string values
---@param var_name string variable name
---@param values table table specifying the allowable values
---@param default_val string  default value
---@param description_val string description to be used in schema
---@param optional boolean whether or not the parameter is optional
---@return string|nil value the value of the parameter, if the parameter is optional and not specified, nil is returned
---
function createEnumeratedParameter(var_name, values, default_val, description_val,optional)
    if not description_val then
        description_val="The description was not defined. Please provide it when defining the input in the task specification lua file with the createEnumeratedParameter function."
    end

    if optional==nil then
        optional=false
    else
        optional=true
    end
    var_optional_tab[var_name] = optional
    table.insert(var_names_tab, var_name)
    if default_val then
        vars_info_tab[var_name] = {
            description = description_val,
            default=default_val,
            oneOf= { { enum= values }}
          }
    else
        vars_info_tab[var_name] = {
            description = description_val,
            oneOf= {{ enum= values }}
          }
    end
    local exists = rawget(_G, var_name) ~= nil
    if exists then
        print(var_name .. " value: " .. rawget(_G, var_name))
        return rawget(_G, var_name)
    else
        if not optional then
            print("Warning: " .. var_name .. " does not exist and thus the default value is used. If this code was executed during generation of JSON Schemas, please ignore this warning.")
        end
        return default_val
    end
end


--- adapt_to_units
---   adapts the units in the table 
---@param tbl  table table of numbers
---@param unit string indicating units ("degrees","radians")
---@return table adapted tbl
function adapt_to_units(tbl, unit)
    if unit=="degrees" then
        for k,v in pairs(tbl) do
            tbl[k] = tbl[k] * math.pi/180.0
        end
    elseif unit=='radians' then
        return tbl
    else
        error("unknown units")
    end
    return tbl
end


-- Later in your Lua script or in an appropriate place, write the vars_info_tab to a file
function write_json_schema(lua_filepath)
    -- local JSON = require("JSON") -- Ensure you have a JSON library like json, dkjson or cjson. In this case just json is used
    
    local filename_lua = lua_filepath:match("^.+/(.+)$")
    local filename_json = filename_lua:gsub("%.lua$", "") ..".json"
    local filename_no_ext = filename_json:gsub("%.json$", ""):gsub("%.etasl$", "")--removes extensions .etasl and .lua
   
    local filepath_lua =  "$[etasl_ros2_application_template]/etasl/task_specifications/" .. filename_lua

    local dkjson = require("dkjson") -- Ensure you have a JSON library like json, dkjson or cjson. In this case dkjson is used
    descript = "Parameters needed to the corresponding task specification in eTaSL"
    if next(var_names_tab) == nil then --Checks if table is empty
        descript = "Parameters needed to the corresponding task specification in eTaSL. In this case no parameters were specified and hence the properties field is empty"
    end
    
    local def_properties = {
        ["is-"..filename_no_ext] = {description="Set to true to indicate that the task specification is defined in: " .. filename_lua .. ". " .. task_description, type="boolean", const=true},
        -- ["full_path_of-"..filename_no_ext] = {description="Full path of the corresponding task specification", enum={lua_filepath}, default=lua_filepath},
        -- ["dependent-parameters"] = {
        --                             ["description"]="List of parameters that need to be defined at runtime (once) depending on e.g. the outcome of a previous action or the outcome of another module (e.g. in a script). Parameters not included in this list are meant to be defined once before startup (e.g. in a JSON file)",
        --                             ["type"]= "array",
        --                             ["uniqueItems"]= true,
        --                             ["minItems"]= 0,
        --                             ["items"]= {
        --                                 ["type"]= "string",
        --                                 ["enum"]= {}
        --                             }
        --                         }
        
        -- ["is-dependent"] = {description="Set to true if the value will be set at runtime (only once) depending on e.g. the outcome of a previous action or the outcome of another module. Set to false if the value of this property is known beforehand and will remain fixed.", type="boolean"}
    }

    
    local schema = {
        ["$schema"]= "http://json-schema.org/draft-06/schema#",
        ["$id"]= filename_no_ext, 
        title= "Task Specification Configuration",
        description = descript,
        type = "object",
        properties = def_properties,
        dependencies = {["is-"..filename_no_ext] ={
            ["properties"]={},
            ["required"]={}
        }},
        required = {"is-"..filename_no_ext},
        additionalProperties= true, --needed to be true for using dependencies
    }

    -- ["full_path_of-"..filename_no_ext] = {description="Full path of the corresponding task specification", enum={lua_filepath}, default=lua_filepath},
    schema.dependencies["is-"..filename_no_ext].properties["file_path"] = {description="File path of the corresponding task specification", type="string", const=filepath_lua}
    table.insert(schema.dependencies["is-"..filename_no_ext].required, "file_path")

    for _, key_name in pairs(var_names_tab) do
        -- print(inspect.inspect(vars_info_tab))schema
        schema.dependencies["is-"..filename_no_ext].properties[key_name] = vars_info_tab[key_name]
        if  (var_optional_tab[key_name]~=nil) and not var_optional_tab[key_name] then
            table.insert(schema.dependencies["is-"..filename_no_ext].required, key_name)
        end
        -- table.insert(schema.properties["dependent-parameters"].items.enum, key_name)
    end

    local file = assert(io.open(filename_json, "w"))
    -- local file = io.open(filename_json, "w")
    -- file:write(JSON:encode(schema))
    file:write(dkjson.encode(schema, { indent = true }))
    file:close()
end

-- max_vel = createScalarParameter("maxvel", 0.65)
-- max_acc = createScalarParameter("maxacc", 0.65)


-- Call this function to write the JSON schema to a file

-- SCRIPT_NAME = string.gsub(string.match(debug.getinfo(1, 'S').short_src, "[^/]+$"), '.lua', '')
-- print(SCRIPT_NAME)

-- write_json_schema("test_schema.json")
-- print("hello")
