local PID = require('pid')
local State = require('state')
local MODE_AUTO = 10
local P = require('geometry/point')
local start_t = millis()
local stage='prepare' -- auto, prepare, pull, para

local roll_controller = PID.new('TRLL', 2.3, 0.2, 0.14, -180, 180)
--local pitch_controller = PID.new('TPIT', 1, 1.6, 0.1, -180, 180)

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            
            local current_state = State.readCurrent()
            
            if current_state:pos():z() > -50 or math.deg(current_state:pitch_angle()) < -50 then
                stage='finshed'
                vehicle:nav_scripting_enable(255)
                --vehicle:set_mode(MODE_AUTO)
                roll_controller:reset()
            else
                local roll = roll_controller:update(0, math.deg(current_state:roll_angle()))
                local pitch = 0.0
                local thr = 50
                if stage=='auto' then
                    start_t=millis()
                    gcs:send_text(6, string.format('LUA: throttle = %i', 20*id))
                    stage='prepare'
                elseif stage=='prepare' then
                    thr = 100
                    pitch = 0 -- pitch_controller:update(0.0, -current_state:att():transform_point(current_state:vel()):z())
                    if millis() - start_t > 3000 then
                        stage = 'pull'
                    end
                elseif stage=='pull' then
                    thr = 100
                    pitch = 20
                    if math.deg(current_state:pitch_angle()) > 30 then
                        stage = 'para'
                    end
                elseif stage=='para' then
                    thr=20 * id
--                    pitch = pitch_controller:update(0.0, -math.deg(current_state:acc():z() / current_state:arspd()))
                    local v = current_state:vel():length()
                    pitch = math.deg(current_state:att():inverse():transform_point(P.z(-9.81)):z() / (v*v) ) 
                end
                
                logger.write('PINF', 'id,cmd,stage', 'iiN', id, cmd, stage )
    
                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, 0.0)
            end
        else
            stage='auto'
        end
    end
    return update, 1000.0/40
end


return update()


