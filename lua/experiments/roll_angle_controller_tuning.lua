
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local roll_controller = require('modules/controllers/roll_angle_controller')
local speed_controller = require('modules/controllers/speed_controller')
local alt_controller = require('modules/controllers/alt_controller')
local sq = SQ.new(0, 5000, 10)

local Exp = {}
function Exp.setup()
    local self = {}
    local state = State.readCurrent()

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            speed_controller:update(0.0, 22, state:flow():length()),
            roll_controller:update(0.0, sq:value(), math.deg(state:roll_angle())),
            alt_controller:update(0.0, 100, -state:pos():z(), math.deg(state:pitch_angle())),
            0
        )
        vehicle:set_rudder_offset(0, true)
    end

    function self:reset()
        sq:reset()
        roll_controller:reset()
        alt_controller:reset()
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
    end

    return self
end



return Exp