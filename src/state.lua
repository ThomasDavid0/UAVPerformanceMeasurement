local g = require('geometry')
local c = require('controller')

local State = {}

function State.new(pos, att, arspd, vel, acc, wind)
    local self = {}
    local _pos = pos
    local _att = att
    local _arspd = arspd
    local _vel = vel
    local _acc = acc
    local _wind = wind

    local _hv = g.quat_earth_to_body(_att, g.makeVector3f(1,0,0))
    local _yaw = math.atan(_hv:y(), _hv:x())
    local _pitch = -math.atan(_hv:z(), math.sqrt(_hv:x() * _hv:x() + _hv:y() * _hv:y()))
    local _roll_angle = g.body_axis_rates(g.qorient(0, _pitch, _yaw), _att):x()
    
    function self.pos()
        return _pos
    end
    function self.att()
        return _att
    end
    function self.arspd()
        return _arspd
    end
    function self.vel()
        return _vel
    end
    function self.acc()
        return _acc
    end
    function self.wind()
        return _wind
    end
    function self.roll_angle()
        return _roll_angle
    end
    
    function self.log(name)
        logger.write(
            name, 'arspd,roll,pitch,yaw', 'ffff',
            _arspd, _roll_angle * 180 / math.pi, _pitch * 180/math.pi , _yaw * 180 / math.pi
        )
    end
    
    return self
end

function State.readCurrent()
    return State.new(
        ahrs:get_relative_position_NED_origin(),
        ahrs:get_quaternion(),
        c.constrain(ahrs:get_EAS2TAS() * math.max(ahrs:airspeed_estimate(), 3), 3, 100),
        ahrs:get_velocity_NED(),
        ahrs:get_accel(),
        ahrs:wind_estimate()
    )
end



return State