

local SQ = {}


function SQ.new(average, period, amplitude)
    local self = {}
    local _average = average
    local _period = period
    local _amplitude = amplitude

    local _side = -1

    local _last_switch = millis()

    function self:value()
        if millis() -  _last_switch > _period then
            _last_switch = millis()
            _side = -_side
        end
        return _average + _amplitude * _side / 2
    end

    function self:reset()
        _last_switch = millis()
        _side = -1
    end
    return self
end

return SQ