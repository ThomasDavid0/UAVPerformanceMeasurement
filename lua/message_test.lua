
local State = require('modules/state')


local _mid = 1

function update()
    if ahrs:get_velocity_NED() == nil then
        return update, 1000.0/40
    end
    
    local state = State.readCurrent()
    if _mid==0 then
        gcs:send_named_float('roll_angle', math.deg(state:roll_angle()))
        _mid = 1
    else
        gcs:send_named_float('pitch_angle', math.deg(state:pitch_angle()))
        _mid=0
    end
    
    
    
    return update, 1000.0/40
end


return update()
