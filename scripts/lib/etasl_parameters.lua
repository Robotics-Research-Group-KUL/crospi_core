require("context")
require("geometric")

-- inspect = require 'inspect'

local vars_info_tab = {}
local var_names_tab = {}
local task_description = ""

-- maxvel = ctx:createInputChannelScalar("maxvel",0.5)

function set_task_description(task_descript)
    task_description = task_descript
end

function createScalarParameter(var_name, default_val, description_val)
    -- local var_tab = {
    --         name=varname,
    --         default=default_val
    -- }
    -- table.insert(vars_info_tab,var_name)
    if not description_val then 
        description_val="The description was not defined. Please provide it when defining the input in the task specification lua file with the createScalarParameter function."
    end

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
        return constant(rawget(_G, var_name))
    else
        print("Warning: " .. var_name .. " does not exist and thus the default value is used. If this code was executed during generation of JSON Schemas, please ignore this warning.")
        return constant(default_val)
    end

    
    -- return ctx:createInputChannelScalar(var_name, default_val)
end

-- Later in your Lua script or in an appropriate place, write the vars_info_tab to a file
function write_json_schema(lua_filepath)
    -- local JSON = require("JSON") -- Ensure you have a JSON library like json, dkjson or cjson. In this case just json is used
    
    local filename_lua = lua_filepath:match("^.+/(.+)$")
    local filename_json = filename_lua:gsub("%.lua$", "") ..".json"
    local filename_no_ext = filename_json:gsub("%.json$", ""):gsub("%.etasl$", "")--removes extensions .etasl and .lua
   
    local filepath_lua =  "$[etasl_ros2_application_template]/etasl/task_specifications/" .. filename_lua

    local dkjson = require("dkjson") -- Ensure you have a JSON library like json, dkjson or cjson. In this case just json is used
    descript = "Parameters needed to the corresponding task specification in eTaSL"
    if next(var_names_tab) == nil then --Checks if table is empty
        descript = "Parameters needed to the corresponding task specification in eTaSL. In this case no parameters were specified and hence the properties field is empty"
    end
    
    local def_properties = {
        ["is-"..filename_no_ext] = {description="Set to true to indicate that the task specification is defined in: " .. filename_lua .. ". " .. task_description, enum={true}},
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
        ["$schema"]= "http://json-schema.org/draft-04/schema#",
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
    schema.dependencies["is-"..filename_no_ext].properties["file_path"] = {description="File path of the corresponding task specification", enum={filepath_lua}}
    table.insert(schema.dependencies["is-"..filename_no_ext].required, "file_path")

    for _, key_name in pairs(var_names_tab) do
        -- print(inspect.inspect(vars_info_tab))
        schema.dependencies["is-"..filename_no_ext].properties[key_name] = vars_info_tab[key_name] 
        table.insert(schema.dependencies["is-"..filename_no_ext].required, key_name)
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