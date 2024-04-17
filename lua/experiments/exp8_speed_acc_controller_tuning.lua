
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local roll_controller = require('modules/controllers/c72_roll_angle_controller')
local acc_controller = require('modules/controllers/c78_thr_acc_controller')
local alt_controller = require('modules/controllers/c74_pitch_alt_controller')

local sq
local istate
local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:reset()
        sq = SQ.new(0, 6000, 1)
        acc_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        alt_controller:reset()
        roll_controller:reset()
        istate=state
    end

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            acc_controller:update(
                0,
                sq:value(),
                state:acc():x()
            ),
            roll_controller:update(0.0, 0, state:roll_angle_deg()),
            alt_controller:update(
                -istate:pos():z(),
                -state:pos():z(),
                state:roll_angle_deg(),
                state:pitch_angle_deg()
            ), 0
        )
        vehicle:set_rudder_offset(0, true)
    end

    return self
end

return Exp


