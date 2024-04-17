local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(76, "SCRTPG_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 2)
local kI = pt:param('kI', 12)
local kD = pt:param('kD', 2)
local min = pt:param('min', -180)
local max = pt:param('max', 180)


local controller = PID.new('TPG', kFF, kP, kI, kD, min, max)

return controller