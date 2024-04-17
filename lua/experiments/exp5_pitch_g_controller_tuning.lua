
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local roll_controller = require('modules/controllers/c72_roll_angle_controller')
local pitch_g_controller = require('modules/controllers/c76_pitch_g_controller')
local speed_controller = require('modules/controllers/c71_speed_controller')
local sq = SQ.new(1, 3000, 1.0)

local istate
local start_t
local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:reset()
        sq:reset()
        pitch_g_controller:reset()
        roll_controller:reset()
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        start_t = millis()
        istate = state
    end

    function self:run()
        if (millis() - start_t) < 7000 then
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(
                    state:pitch_angle_deg(),
                    istate:arspd(),
                    state:arspd()
                ),
                roll_controller:update(0.0, 0, state:roll_angle_deg()),
                pitch_g_controller:update(0, sq:value(), -state:acc():z()/9.81),
                0
            )
            vehicle:set_rudder_offset(0, true)
        end
    end

    return self
end

return Exp