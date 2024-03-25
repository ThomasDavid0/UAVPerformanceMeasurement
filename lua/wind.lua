local flightmodes = require('modules/mappings/modes')
local State = require('modules/state')

function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local state = State.readCurrent()
        gcs:send_named_float("st_arspd", state:arspd())
        gcs:send_named_float("st_flow", state:flow():length())
        gcs:send_named_float("st_w_spd", state:wind():length())
    end
    return update, 250
end

return update()