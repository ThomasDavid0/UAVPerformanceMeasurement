
local P = require('modules/geometry/point')
local Q = require('modules/geometry/quaternion')
local PID = require('modules/controllers/pid')

local State = {}

function State.new(pos, att, arspd, vel, acc, wind)
    local self = {}
    local _pos = pos
    local _att = att
    local _arspd = arspd
    local _vel = vel
    local _acc = acc
    local _wind = wind

    local _body_wind = _att:inverse():transform_point(_wind)
    local _flow = P.sub(_vel, _body_wind)
    local _hv = _att:transform_point(P.xyz(1,0,0))
    local _yaw = math.atan(_hv:y(), _hv:x())
    local _pitch = -math.atan(_hv:z(), math.sqrt(_hv:x() * _hv:x() + _hv:y() * _hv:y()))
    local _roll_angle = Q.body_axis_rates(Q.euler(P.xyz(0, _pitch, _yaw)), _att):x()
    

    function self:pos()
        return _pos
    end
    function self:att()
        return _att
    end
    function self:arspd()
        return _arspd
    end
    function self:vel()
        return _vel
    end
    function self:acc()
        return _acc
    end
    function self:wind()
        return _wind
    end
    function self:roll_angle()
        return _roll_angle
    end
    function self:roll_angle_deg()
        return math.deg(_roll_angle)
    end
    function self:pitch_angle()
        return _pitch
    end
    function self:pitch_angle_deg()
        return math.deg(_pitch)
    end

    function self:flow()
        return _flow
    end

    function self:log(name)
        logger.write(
            name, 'arspd,roll,pitch,yaw', 'ffff',
            _arspd, _roll_angle * 180 / math.pi, _pitch * 180 / math.pi , _yaw * 180 / math.pi
        )
    end
    
    return self
end

function State.readCurrent()
    local pos = P.new(ahrs:get_relative_position_NED_origin())
    local att = Q.new(ahrs:get_quaternion())
    local atti = att:inverse()
    return State.new(
        pos,
        att,
        PID.constrain(ahrs:get_EAS2TAS() * math.max(ahrs:airspeed_estimate(), 3), 3, 100),
        atti:transform_point(P.new(ahrs:get_velocity_NED())),
        P.new(ahrs:get_accel()),
        P.new(ahrs:wind_estimate())
    )
end



return State