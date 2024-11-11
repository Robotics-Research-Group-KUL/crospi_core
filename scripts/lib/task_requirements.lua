-- ==============================================================================
-- Author: Santiago Iregui
-- email: <santiago.iregui@kuleuven.be>
-- Code for defining etasl properties and automatically generating JSON schemas
-- KU Leuven 2024
-- ==============================================================================
local M = {}
M.params = {}
M.robot = {}

--- Table defining allowed types for enums.
M.enum_types = {
    number="number",
    integer="integer",
    string="string",
}

--- Table defining allowed types for arrays.
M.array_types = {
    number="number",
    integer="integer",
    bool="bool",
    string="string",
}

-- inspect = require("inspect")

local task_req = {}
task_req.parameters = {}

--- table to record wheter a parameter is required
local required_parameters = {}

--- Processes an input table with specified properties for generic parameters (i.e. all parameters use this function).
-- This function is meant to be used locally (i.e. it is not exposed to the user)
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (number): The default value of the specified parameter. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
-- @return param_schema table A table containing a partial JSON schema that later on will be completed
local function param_generic(spec)

    -- if not spec.description then
    --     spec.description="The description was not defined. Please provide it when defining the input in the task specification lua file"
    -- end

    local param_schema = {}

    if (not spec.name) or spec.name == "" then
        error("The defined parameter requires a non-empty name")
    end

    if (not spec.description) or spec.description == "" then
        error("The defined parameter requires a non-empty description")
    end

    if not spec.default then
        error("The defined parameter requires a default value")
    end

    if spec.required==nil or spec.required == true then        
        table.insert(required_parameters, spec.name)
    end

    param_schema[spec.name] = {
        description = spec.description .. ". Set as 'external' if the value is not yet known and thus will be set externally at runtime (only once) depending on e.g. the outcome of a previous action or the outcome of another module.",
    }
    
    -- If default value is specified, it is added to the schema
    if spec.default then
        -- table.insert(param_schema[spec.name], "default")
        param_schema[spec.name].default = spec.default
    end
    
    return param_schema
    
end

--- Processes an input table with specified properties for generic parameters (i.e. all numerical parameters, i.e. number and integer, use this function).
--- It's meant to be used locally, i.e. it is not exposed to the user.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (number): The default value of the specified parameter. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--  - `minimum` (number): Minimum value allowed. Optional.
--  - `maximum` (number): Maximum value allowed. Optional.
--- @param schema (table) A table containing a partial JSON schema defining the parameter
--- @return param_schema table A table containing a partial JSON schema defining the parameter
--- @return tab_fields table A table containing a partial JSON schema defining specific properties of the numerical parameter
local function param_numerical(spec, schema)

    local param_schema = schema --pre-fills generic data necessary by all types

    local tab_fields = {}

    if spec.minimum~=nil then
        if type(spec.minimum) == "number" then
            tab_fields.minimum = spec.minimum
        else
            error("The minimum value specified for parameter " .. spec.name .. " should be a number.")
        end
    end

    if spec.maximum~=nil then
        if type(spec.maximum) == "number" then
            tab_fields.maximum = spec.maximum
        else
            error("The maximum value specified for parameter " .. spec.name .. " should be a number.")
        end
    end

    if spec.default and type(param_schema[spec.name].default) ~= "number" then
        error("The default value specified for parameter " .. spec.name .. " should be a number.")
    end
    if spec.maximum and spec.default > spec.maximum then
        error("The specified default value of the parameter " .. spec.name .. " is above the specified maximum value")
    end
    if spec.minimum and spec.default < spec.minimum then
        error("The specified default value of the parameter " .. spec.name .. " is below the specified minimum value")
    end



    return param_schema, tab_fields
end


--- Validates the allowed specs, such that only certain table entries are allowed.
--- It's meant to be used locally, i.e. it is not exposed to the user
--- @param allowed_specs (table) A table containing the allowed entries when defining a parameter.
--- @param spec (table) A table containing the specification of the parameter, which should not have any entries apart from the ones defined in allowed_specs
--- @param input_type (string) A string describing the input_type for logging purposes.
--- @return nil
local function validate_allowed_specs(allowed_specs, spec, input_type)
    for key_spec, _ in pairs(spec) do
        local is_valid = false
        for _, key_allowed_specs in pairs(allowed_specs) do
            if key_allowed_specs == key_spec then
                is_valid = true
                break
            end
        end
        if not is_valid then
            local allowed_specs_string = ""
            for _, key_allowed_specs in pairs(allowed_specs) do
                allowed_specs_string = allowed_specs_string .. key_allowed_specs .. ", "
            end
            error("The input `".. key_spec .."` is not valid when defining parameters of type " .. input_type ..". The only allowed inputs are: " .. allowed_specs_string)
        end
    end

    
end

--- Defines a scalar parameter.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (number): The default value of the specified parameter. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--  - `minimum` (number): Minimum value allowed. Optional.
--  - `maximum` (number): Maximum value allowed. Optional.
--- @return param_schema table A table containing a JSON schema defining the parameter
local function param_scalar(spec)
    local allowed_specs = {"name","description","default","required","minimum","maximum"}
    validate_allowed_specs(allowed_specs, spec, "number")

    local param_schema = param_generic(spec) --pre-fills generic data necessary by all types
    param_schema, tab_fields = param_numerical(spec, param_schema) -- Fills fields required for all numbers (double and integers)
    tab_fields.type = "number"
    param_schema[spec.name].oneOf = {tab_fields, { enum= {"external"}}}
    return param_schema
end


--- Defines an integer parameter.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (number): The default value of the specified parameter. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--  - `minimum` (number): Minimum value allowed. Optional.
--  - `maximum` (number): Maximum value allowed. Optional.
--- @return param_schema table A table containing a JSON schema defining the parameter
local function param_int(spec)
    local allowed_specs = {"name","description","default","required","minimum","maximum"}
    validate_allowed_specs(allowed_specs, spec, "integer")

    local param_schema = param_generic(spec) --pre-fills generic data necessary by all types
    param_schema, tab_fields = param_numerical(spec, param_schema) -- Fills fields required for all numbers (double and integers)
    tab_fields.type = "integer"
    param_schema[spec.name].oneOf = {tab_fields, { enum= {"external"}}}
    return param_schema
end

--- Defines a boolean parameter.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (number): The default value of the specified parameter. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--- @return param_schema table A table containing a JSON schema defining the parameter
local function param_bool(spec)
    local allowed_specs = {"name","description","default","required"}
    validate_allowed_specs(allowed_specs, spec, "boolean")

    local param_schema = param_generic(spec) --pre-fills generic data necessary by all types
    if spec.default and type(param_schema[spec.name].default) ~= "boolean" then
        error("The default value specified for parameter " .. spec.name .. " should be a boolean.")
    end    
    param_schema[spec.name].oneOf = {{ type= "boolean" }, { enum= {"external"}}}
    return param_schema
end


--- Defines a enum parameter.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (type): The default value of the specified parameter. Must belong to the accepted_vals and belong to the specified type. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--  - `type` (string): String specifying the type of enum. The allowed types are number, integer and string. Required.
--  - `accepted_vals` (table): A table that contains the accepted values of the enum. Required.
--- @return param_schema table A table containing a JSON schema defining the parameter
local function param_enum(spec)
    local allowed_specs = {"name","description","default","required", "type","accepted_vals"}
    validate_allowed_specs(allowed_specs, spec, "enum")

    local param_schema = param_generic(spec) --pre-fills generic data necessary by all types
    -- task_req.parameter.enum({name="units", type=types.string, default="radians", description="Description of units", required=true, accepted_vals = {"degrees","radians"}}),
    
    -- param_schema[spec.name].type = spec.type

    if not spec.type then
        error("The type for enum parameter " .. spec.name .. " was not specified. The valid types for are: number, integer and string.")
    elseif spec.type~= "number" and spec.type~= "integer" and spec.type~= "string" then
        error("Incorrect specified type for enum parameter " .. spec.name .. ". The only valid types for enum parameters are: number, integer and string. The specified type was: " .. spec.type.. ".")
    end

    
    if not spec.accepted_vals then
        error("The accepted_vals for enum parameter " .. spec.name .. " was not specified. The accepted_vals must be of the same type of enum specified")
    else
        for _, value in pairs(spec.accepted_vals) do
            if type(value) ~= spec.type then
                error("One of the speicified accepted_vals of the enum " .. spec.name .. " is not of the same type as the defined type.")
            end
        end
    end

    if spec.default and type(param_schema[spec.name].default) ~= spec.type then -- The requirement of spec.default is handled in param_numerical function, thus here we validate if it's defined
        error("The default value specified for parameter " .. spec.name .. " should be of the same type of the specified type, i.e." .. spec.type.. ".")
    end

    local default_is_valid = false
    for _, value in pairs(spec.accepted_vals) do
        if spec.default == value then
            default_is_valid = true
            break
        end
    end
    if not default_is_valid then
        error("The specified default value of the enum " .. spec.name .. " does not correspond to one of the specified accepted_vals")
    end
    
    param_schema[spec.name].oneOf = {{ enum= spec.accepted_vals }, { enum= {"external"}}}

    return param_schema
end


--- Defines an array parameter.
--- @param spec (table) A table containing the specification of the parameter with specific keys and values:
--  - `name` (string): The name of the specified parameter. Required.
--  - `description` (string): The description of the specified parameter, which serves for documentation. Required.
--  - `default` (type): The default value of the specified parameter. Must belong to the accepted_vals and belong to the specified type. Required.
--  - `required` (boolean): Indicates if the parameter is required. Set as true if not specified. Optional.
--  - `type` (string): String specifying the type of enum. The allowed types are number, integer and string. Required.
--  - `minimum` (number): Minimum value allowed. Optional.
--  - `maximum` (number): Maximum value allowed. Optional.
--- @return param_schema table A table containing a JSON schema defining the parameter
local function param_array(spec)
    local allowed_specs = {"name","description","default","required", "type","minimum", "maximum"}
    validate_allowed_specs(allowed_specs, spec, "array")

    local param_schema = param_generic(spec) --pre-fills generic data necessary by all types
    -- task_req.parameter.array({name="units", type=types.string, default="radians", description="Description of units", required=true, accepted_vals = {"degrees","radians"}}),
    
    -- param_schema[spec.name].items = {}
    -- param_schema[spec.name].items.type = spec.type

    if not spec.type then
        error("The type for array parameter " .. spec.name .. " was not specified. The valid types for are: number, integer, string and boolean.")
    elseif spec.type ~= "number" and spec.type ~= "integer" and spec.type ~= "string" and spec.type ~= "boolean" then
        error("Incorrect specified type for array parameter " .. spec.name .. ". The only valid types for array parameters are: number, integer, string and boolean. The specified type was: " .. spec.type .. ".")
    end
    

    if spec.minimum~=nil and (spec.type ~= "number" and spec.type ~= "integer") then
        error("No minimum value is allowed to be specified since the type of the array " .. spec.name .. " is not number or integer.")
    end

    if spec.maximum~=nil and (spec.type ~= "number" and spec.type ~= "integer") then
        error("No maximum value is allowed to be specified since the type of the array " .. spec.name .. " is not number or integer.")
    end
    local tab_fields = { type= "array", items={type=spec.type} }

    if spec.minimum~=nil then
        if type(spec.minimum) == "number" then
            tab_fields.minItems = spec.minimum
        else
            error("The minimum value specified for parameter " .. spec.name .. " should be a number.")
        end
    end
    
    if spec.maximum~=nil then
        if type(spec.maximum) == "number" then
            tab_fields.maxItems = spec.maximum
        else
            error("The maximum value specified for parameter " .. spec.name .. " should be a number.")
        end
    end

    if not spec.default then
        error("The default value for array parameter " .. spec.name .. " was not specified. The elements of the default array must be of the same type of array specified")
    else
        for _, value in pairs(spec.default) do
            if type(value) ~= spec.type then
                error("One of the speicified default elements of the array parameter " .. spec.name .. " is not of the same type as the defined type.")
            end
            if spec.maximum and value > spec.maximum then
                error("One of the speicified default elements of the array parameter " .. spec.name .. " is above the specified maximum value")
            end
            if spec.minimum and value < spec.minimum then
                error("One of the speicified default elements of the array parameter " .. spec.name .. " is below the specified minimum value")
            end
        end
    end


    param_schema[spec.name].oneOf = {tab_fields, { enum= {"external"}}}

    return param_schema
end

--- Generates a JSON Schema file based on the parameters defined.
--- It's meant to be used locally, i.e. it is not exposed to the user
--- @param task_description (string) A string containing a description of the task specification.
--- @param param_tab (table) A table containing the JSON Schema specification of all individual parameters.
--- @return nil
local function write_json_schema(task_description, param_tab)

    local filename_lua = LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA:match("^.+/(.+)$") --obtains the filename from a full path
    local filename_json = filename_lua:gsub("%.lua$", "") ..".json" --removes the .lua extension and adds the .json for the generated schema
    local filename_no_ext = filename_json:gsub("%.json$", ""):gsub("%.etasl$", "")--removes extensions .etasl and .lua

    local descript = "Parameters needed to the corresponding task specification in eTaSL"
    if next(param_tab) == nil then --Checks if table is empty
        descript = "Parameters needed to the corresponding task specification in eTaSL. In this case no parameters were specified and hence the properties field is empty"
    end
    
    local def_properties = {
        ["is-"..filename_no_ext] = {description="Set to true to indicate that the task specification is defined in: " .. filename_lua .. ". " .. task_description, type="boolean", const=true},
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

    local filepath_lua =  "$[etasl_ros2_application_template]/etasl/task_specifications/" .. filename_lua
    schema.dependencies["is-"..filename_no_ext].properties["file_path"] = {description="File path of the corresponding task specification", type="string", const=filepath_lua}

    for k, _ in ipairs(param_tab) do
        local key_name = next(param_tab[k])
        schema.dependencies["is-"..filename_no_ext].properties[key_name] = param_tab[k][key_name]
    end
    schema.dependencies["is-"..filename_no_ext].required = required_parameters
    table.insert(schema.dependencies["is-"..filename_no_ext].required, "file_path")


    -- Save the JSON Schema in a pretty format
    local file = assert(io.open(filename_json, "w"))
    local dkjson = require("dkjson") -- Ensure you have a JSON library like json, dkjson or cjson. In this case dkjson is used
    file:write(dkjson.encode(schema, { indent = true }))
    file:close()
end

--- Generates a JSON Schema file based on the parameters defined.
--- @param task_description (string) A string containing a description of the task specification.
--- @param param_tab (table) A table containing the JSON Schema specification of all individual parameters.
--- @return nil
local function parameters(task_description, param_tab)
    -- for k, _ in ipairs(param_tab) do
    --     local key_name = next(param_tab[k])
    --     print(key_name .. " = " .. inspect(param_tab[k][key_name]))
    --     print("-----------------------------")
    -- end

    if LUA_FILEPATH_TO_GENERATE_JSON_SCHEMA then --this variable should be defined only during generation of JSON Schema
        write_json_schema(task_description, param_tab)
    end
end



--- adapt_to_units
---   adapts the units in the table 
---@param tbl (table) table of numbers
---@param unit (string) indicating units ("degrees","radians")
---@return (table) adapted tbl
local function adapt_to_units(tbl, unit)
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


M.parameters = parameters
M.adapt_to_units= adapt_to_units
M.params.scalar = param_scalar
M.params.int= param_int
M.params.bool= param_bool
M.params.enum= param_enum
M.params.array= param_array


return M
