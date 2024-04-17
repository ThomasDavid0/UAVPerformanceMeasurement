
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local controllers = require('modules/controllers/controllers')
--local roll_controller = require('modules/controllers/c72_roll_angle_controller')
--local pitch_controller = require('modules/controllers/c73_pitch_angle_controller')
--local speed_controller = require('modules/controllers/c71_speed_controller')
local sq = SQ.new(2, 6000, 10)

local istate

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:reset()
        sq:reset()
        controllers.reset_all(
            {TSPD=SRV_Channels:get_output_scaled(functions.throttle)}
        )
        --pitch_controller:reset()
        --roll_controller:reset()
        --speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        istate = state
    end

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            controllers.TSPD:update(
                state:pitch_angle_deg(),
                istate:arspd(),
                state:arspd()
            ),
            controllers.TRAN:update(0.0, 0, state:roll_angle_deg()),
            controllers.TPAN:update(0.0, sq:value(), state:pitch_angle_deg()),
            0
        )
        vehicle:set_rudder_offset(0, true)
    end

    return self
end

return Exp