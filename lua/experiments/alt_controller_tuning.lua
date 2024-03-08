
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local alt_controller = require('modules/controllers/alt_controller')
local sq = SQ.new(100, 10000, 5)

local Exp = {}
function Exp.setup()
    local self = {}
    local state = State.readCurrent()

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            SRV_Channels:get_output_scaled(functions.throttle),
            0,
            alt_controller:update(0.0, sq:value(), -state:pos():z(), math.deg(state:pitch_angle())),
            0
        )
        vehicle:set_rudder_offset(0, true)
    end

    function self:reset()
        sq:reset()
        alt_controller:reset()
    end

    return self
end

return Exp