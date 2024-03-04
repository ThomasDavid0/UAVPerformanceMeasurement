local PID = require('controllers/pid')
local PT = require('param_table')

local pt = PT:new(71, "SCRTSPD_", 6)

local speed_controller = PID.new(
    'TSPD',
    pt:param('kFF', 1):get(),
    pt:param('kP', 60):get(),
    pt:param('kI', 50):get(),
    pt:param('kD', 0):get(),
    pt:param('min', 0):get(),
    pt:param('max', 100):get()
)

return speed_controller