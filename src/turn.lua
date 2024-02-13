
local comms = require('comms')

local Turn = {}


function Turn.new(id, load_factor, alt, arspd)
    local self = {}
    local _id = id
    local _load_factor = load_factor
    local _alt = alt
    local _stage = 0
    local _arspd = arspd
    local _roll_angle = math.acos(1 / _load_factor)
    local _start_time = millis() / 1000


    function self:arspd()
        return _arspd
    end
    function self:id()
        return _id
    end
    function self:load_factor()
        return _load_factor
    end
    function self:alt()
        return _alt
    end
    function self:stage()
        return _stage
    end
    function self:stagestring()
        return string.format('STG%i',_stage)
    end
    function self:next_stage()
        _stage = _stage + 1
    end
    function self:roll_angle()
        return _roll_angle
    end
    function self:start_time()
        return _start_time
    end
    function self:summary()
        return string.format("Turn: %i, target g: %f, roll angle: %f", _id, _load_factor, math.deg(_roll_angle))
    end  

    return self
end


function Turn.initialise(id, cmd)
    if cmd == 1 then
        local new_turn = Turn.new(id, 1.1 + 0.2*id, ahrs:get_relative_position_NED_origin():z(), ahrs:airspeed_estimate())
        comms.gcsWrite(new_turn:summary())
        return new_turn
    end
end

function Turn.timout(active_turn)
    if active_turn then
        comms.gcsWrite(string.format("timeout %s",active_turn.summary()))
        return nil
    end
end


return Turn