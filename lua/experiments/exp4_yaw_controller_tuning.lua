
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local roll_controller = require('modules/controllers/roll_angle_controller')
local speed_controller = require('modules/controllers/speed_controller')
local alt_controller = require('modules/controllers/alt_controller')
local yaw_controller = require('modules/controllers/yaw_controller')
local sq = SQ.new(0, 5000, 2)

local Exp = {}
local initial_state=nil
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:run()
        if initial_state then
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, 22, state:flow():length()),
                roll_controller:update(0.0, 0, math.deg(state:roll_angle())),
                alt_controller:update(
                    -initial_state:pos():z(),
                    -state:pos():z(),
                    state:roll_angle(),
                    state:pitch_angle()
                ),
                yaw_controller:update(0.0, sq:value(), -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)
        end
    end

    function self:reset()
        sq:reset()
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        alt_controller:reset()
        roll_controller:reset()
        yaw_controller:reset()
        initial_state = state
    end

    return self
end

return Exp


