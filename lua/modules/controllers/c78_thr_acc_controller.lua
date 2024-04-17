local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(78, "SCRTSPA_", 6)

local kFF = pt:param('kFF', 1.2)
local kP = pt:param('kP', 5.0)
local kI = pt:param('kI', 0.0)
local kD = pt:param('kD', 0)
local min = pt:param('min', 0)
local max = pt:param('max', 100)


local speed_controller = PID.new('TSPA', kFF, kP, kI, kD, min, max)

return speed_controller