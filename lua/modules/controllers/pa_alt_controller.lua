local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(74, "SCRTPALT_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 2.0)
local kI = pt:param('kI', 0.0)
local kD = pt:param('kD', 0.0)
local min = pt:param('min', -40)
local max = pt:param('max', 40)


local controller = PID.new('TPAL', kFF, kP, kI, kD, min, max)

return controller