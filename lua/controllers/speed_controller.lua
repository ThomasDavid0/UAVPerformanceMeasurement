local PID = require('controllers/pid')

local speed_controller = PID.new('TSPD', 60, 50, 0, 0, 100)

return speed_controller