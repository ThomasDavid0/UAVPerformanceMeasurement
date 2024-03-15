
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local pitch_controller = require('modules/controllers/pitch_angle_controller')
local sq = SQ.new(2, 5000, 5)

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            SRV_Channels:get_output_scaled(functions.throttle),
            0,
            pitch_controller:update(0.0, sq:value(), math.deg(state:pitch_angle())),
            0
        )
        vehicle:set_rudder_offset(0, true)
    end

    function self:reset()
        sq:reset()
        pitch_controller:reset()
    end
    return self
end

return Exp