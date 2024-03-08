local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(71, "SCRTSPD_", 6)

local kFF = pt:param('kFF', 1)
local kP = pt:param('kP', 60)
local kI = pt:param('kI', 50)
local kD = pt:param('kD', 0)
local min = pt:param('min', 0)
local max = pt:param('max', 100)


local speed_controller = PID.new('TSPD', kFF, kP, kI, kD, min, max)

return speed_controller