--- Basic type convertion from lua tables to etasl structures and vice-versa
--- E. Aertbelien 2023.

require("context")
local M = {}

---Convert an eTaSL Vector to a table
---@param v Vector
---@return table
function M.Vector_to_table(v)
    return { v:x(), v:y(), v:z() }
end

---convert a table to an eTaSL Vector
---@param tbl table
---@return Vector
function M.Vector_from_table(tbl)
    return Vector(tbl[1],tbl[2],tbl[3])
end

---convert an eTaSL Rotation to a table
---@param R Rotation
---@return table
function M.Rotation_to_table(R)
    return {
            M.Vector_to_table(R:UnitX()),
            M.Vector_to_table(R:UnitY()),
            M.Vector_to_table(R:UnitZ())}
end

function M.Rotation_from_table(tbl)
    return Rotation(
        M.Vector_from_table(tbl[1]),
        M.Vector_from_table(tbl[2]),
        M.Vector_from_table(tbl[3]) )
end

function M.Frame_from_table(tbl)
    return Frame(
        M.Rotation_from_table(tbl[1]),
        M.Vector_from_table(tbl[2]))
end

function M.Frame_to_table(R)
    return {
        M.Rotation_to_table(R:Rotation()),
        M.Vector_to_table(R:Origin())
        }
end


return M