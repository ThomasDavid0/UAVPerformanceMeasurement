
local pitch_controller = require('modules/controllers/pitch_angle_controller')
local pa_alt_controller = require('modules/controllers/pa_alt_controller')

local cnt = {}


function cnt:update(target_alt, current_alt, current_roll, current_pitch)
    local ff = -1 / math.cos(math.rad(current_roll))
    local pitch_target = pa_alt_controller:update(ff, target_alt, current_alt)

    return pitch_controller:update(0, pitch_target, math.deg(current_pitch))
end

function cnt:reset()
    pitch_controller:reset()
    pa_alt_controller:reset()
end


return cnt