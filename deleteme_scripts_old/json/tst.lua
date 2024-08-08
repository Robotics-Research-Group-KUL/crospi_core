JSON = require("JSON")

function readjson(filename)
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

param=readjson("tst.json")

print( param.frequency)
print( param.amplitude)

