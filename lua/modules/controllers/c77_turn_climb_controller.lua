local roll_controller = require('modules/controllers/c72_roll_angle_controller')

local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(77, "SCRTRCLB_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 0.1)
local kI = pt:param('kI', 0.1)
local kD = pt:param('kD', 0.0)
local min = pt:param('min', 5)
local max = pt:param('max', 85)

local controller = PID.new('TRCL', kFF, kP, kI, kD, min, max)

local cnt = {}

function cnt:update(target_climb_rate, current_climb_rate, current_roll_angle)
    local target_roll_angle = controller:update(0, target_climb_rate, current_climb_rate)
    return roll_controller:update(0, target_roll_angle, current_roll_angle)
end

function cnt:reset(value)
    roll_controller:reset()
    controller:reset()
end


return cnt