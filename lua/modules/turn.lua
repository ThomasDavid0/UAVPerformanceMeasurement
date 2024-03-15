

local Turn = {}


function Turn.new(id, load_factor, alt, arspd)
    local self = {}
    local _load_factor = load_factor
    local _alt = alt
    local _stage = 0
    local _arspd = arspd
    local _roll_angle = math.acos(1 / _load_factor)
    local _start_time = millis() / 1000
    
    function self:arspd()
        return _arspd
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
        return string.format(
            "target g: %f, roll angle: %f, arspd= %f",
            _load_factor,
            math.deg(_roll_angle),
            _arspd
        )
    end

    gcs:send_text(6, self:summary())

    return self
end


function Turn.timout(active_turn)
    if active_turn then
        gcs:send_text(6, string.format("timeout %s",active_turn.summary()))
        return nil
    end
end


return Turn