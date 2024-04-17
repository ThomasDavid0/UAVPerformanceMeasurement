
local pitch_controller = require('modules/controllers/c73_pitch_angle_controller')

local PID = require('modules/controllers/pid')
local PT = require('modules/param_table')

local pt = PT.new(74, "SCRTPALT_", 6)

local kFF = pt:param('kFF', 0)
local kP = pt:param('kP', 2.2)
local kI = pt:param('kI', 0.1)
local kD = pt:param('kD', 0.0)
local min = pt:param('min', -30)
local max = pt:param('max', 30)

local controller = PID.new('TPAL', kFF, kP, kI, kD, min, max)

local cnt = {}

function cnt:update(target_alt, current_alt, current_roll, current_pitch)
    local pitch_target = controller:update(
        -1 / math.cos(math.rad(current_roll)),
        target_alt, current_alt
    )
    return pitch_controller:update(0, pitch_target, current_pitch)
end

function cnt:reset(value)
    pitch_controller:reset()
    controller:reset()
end

function cnt:angle_controller()
    return controller
end

function cnt:pitch_controller()
    return pitch_controller
end


return cnt