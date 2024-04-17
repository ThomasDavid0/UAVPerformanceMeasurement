
local State = require('modules/state')

local roll_controller = require('modules/controllers/c72_roll_angle_controller')
local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
local yaw_controller = require('modules/controllers/c75_yaw_controller')
local pitch_g_controller = require('modules/controllers/c76_pitch_g_controller')

local stage
local istate
local timer

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()
    
    function self:reset()
        pitch_g_controller:reset()
        roll_controller:reset()
        yaw_controller:reset()
        stage = 0
        istate = state
        timer = millis()
    end

    function self:run()
        local _thr = 100
        local _pitch = 0
        
        if stage == 0 then
            _pitch = alt_controller:update(
                -istate:pos():z(),
                -state:pos():z(),
                state:roll_angle_deg(),
                state:pitch_angle_deg()
            )
            if timer + 5000 < millis() then
                stage = 1
            end
        elseif stage == 1 then
            _pitch=40
            if state:pitch_angle_deg() > 20 then
                stage = 2
            end
        elseif stage==2 then
            _pitch = pitch_g_controller:update(0, 0, -state:acc():z()/9.81)
            _thr=80
            if -state:pos():z() < (-istate:pos():z() - 20) then
                stage = 3
            end
        elseif stage==3 then
            _pitch = 40
            _thr=20
            if state:att():transform_point(state:vel()):z() < 0 then
                stage = 4
            end
        end
        if stage < 4 then
            vehicle:set_target_throttle_rate_rpy(
                _thr,
                roll_controller:update(0, 0, state:roll_angle_deg()),
                _pitch,
                yaw_controller:update(0.0, 0.0, -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)    
        end
        
    end

    return self
end

return Exp


