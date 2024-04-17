local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(73, "SCRTPAN_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 2)
local kI = pt:param('kI', 0.3)
local kD = pt:param('kD', 0.0)
local min = pt:param('min', -90)
local max = pt:param('max', 90)


local controller = PID.new('TPAN', kFF, kP, kI, kD, min, max)

return controller