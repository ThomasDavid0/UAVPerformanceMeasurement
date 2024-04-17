local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(72, "SCRTRAN_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 5.0)
local kI = pt:param('kI', 1.0)
local kD = pt:param('kD', 2.0)
local min = pt:param('min', -90)
local max = pt:param('max', 90)


local controller = PID.new('TRAN', kFF, kP, kI, kD, min, max)

return controller