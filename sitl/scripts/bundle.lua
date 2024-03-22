local loaded_files = {}
local files = {}
local require = function(name)
    if loaded_files[name] == nil then
        loaded_files[name] = files[name]()
    end
    return loaded_files[name]
end
files['modules/mappings/modes'] = function(...)
    local flightmodes = {
        MANUAL = 0,
        CIRCLE = 1,
        STABILIZE = 2,
        TRAINING = 3,
        ACRO = 4,
        FBWA = 5,
        FBWB = 6,
        CRUISE = 7,
        AUTOTUNE = 8,
        AUTO = 10,
        RTL = 11,
        LOITER = 12,
        TAKEOFF = 13,
        AVOID_ADSB = 14,
        GUIDED = 15,
        QSTABILIZE = 17,
        QHOVER = 18,
        QLOITER = 19,
        QLAND = 20,
        QRTL = 21,
        QAUTOTUNE = 22,
        QACRO = 23,
        THERMAL = 24,
        LOITER_TO_QLAND = 25,
    }
    
    return flightmodes
end
files['modules/geometry/point'] = function(...)
    
    
    local Point = {}
    
    function Point.new(p)
        local self = {}
        local _p = p
    
        function self:x()
            return _p:x()
        end
        function self:y()
            return _p:y()
        end
        function self:z()
            return _p:z()
        end
        function self:copy()
            return Point.xyz(_p:x(), _p:y(), _p:z())
        end
        function self:p()
            return _p
        end
    
        function self:length()
            return _p:length()
        end
    
        function self:scale(value)
            return Point.xyz(_p:x() * value, _p:y() * value, _p:z() * value)
        end
    
        function self:unit()
            return Point.new(self:scale(1/self:length()))
        end
    
        return self
    end
    
    function Point.xyz(x, y, z)
        local vo = Vector3f()
        vo:x(x)
        vo:y(y)
        vo:z(z)
        return Point.new(vo)
    end
    
    function Point.add(p1, p2)
        return Point.xyz(p1:x() + p2:x(), p1:y() + p2:y(), p1:z() + p2:z())
    end
    
    function Point.sub(p1, p2)
        return Point.xyz(p1:x() - p2:x(), p1:y() - p2:y(), p1:z() - p2:z())
    end
    
    function Point.dot(p1, p2)
        return p1:x()*p2:x() + p1:y()*p2:y() + p1:z()*p2:z()
    end
    
    function Point.cross(p1, p2)
        return Point.new(p1:p():cross(p2:p()))
    end
    function Point.x(v)
        return Point.xyz(v, 0, 0)
    end
    function Point.y(v)
        return Point.xyz(0, v, 0)
    end
    function Point.z(v)
        return Point.xyz(0, 0, v)
    end
    
    
    
    return Point
end
files['modules/geometry/quaternion'] = function(...)
    
    local Point = require('modules/geometry/point')
    
    local Quat = {}
    
    
    function Quat.new(q)
        local self = {}
        local _q = q
    
        function self:w()
            return _q:q1()
        end
        function self:x()
            return _q:q2()
        end
        function self:y()
            return _q:q3()
        end
        function self:z()
            return _q:q4()
        end
        function self:q1()
            return _q:q1()
        end
        function self:q2()
            return _q:q2()
        end
        function self:q3()
            return _q:q3()
        end
        function self:q4()
            return _q:q4()
        end
        function self:q()
            return _q
        end
        function self:normalize()
            local qo = self:copy():q()
            qo:normalize()
            return Quat.new(qo)
        end
        function self:copy()
            return Quat.wxyz(_q:q1(), _q:q2(), _q:q3(), _q:q4())
        end
        function self:inverse()
            return Quat.new(_q:inverse())
        end
        function self:transform_point(v)
            local vo = v:copy():p()
            _q:earth_to_body(vo)
            return Point.new(vo)
        end
        function self:axis()
            return Point.xyz(_q:q2(), _q:q3(), _q:q4())
        end
        function self:conjugate()
            return Quat.wxyz(_q:q1(), -_q:q2(), -_q:q3(), -_q:q4())
        end
        function self:to_axis_angle()
            local vo = Vector3f()
            _q:to_axis_angle(vo)
            return Point.new(vo)
        end
        return self
    end
    
    function Quat.wxyz(w, x, y, z)
        local q = Quaternion()
        q:q1(w)
        q:q2(x)
        q:q3(y)
        q:q4(z)
        return Quat.new(q)
    end
    function Quat.euler(p)
        local _q = Quaternion()
        _q:from_euler(p:x(), p:y(), p:z())
        return Quat.new(_q)
    end
    function Quat.euldeg(p)
        return Quat.euler(Point.xyz(math.rad(p:x()), math.rad(p:y()), math.rad(p:z())))
    end
    function Quat.mul(a, b)
        local w = a:w() * b:w() - Point.dot(a:axis(), b:axis())
        local xyz = Point.add(Point.add(b:axis():scale(a:w()), a:axis():scale(b:w())), Point.cross(a:axis(), b:axis()))
    
        return Quat.wxyz(w, xyz:x(), xyz:y(), xyz:z())
    end
    function Quat.body_axis_rates(a, b)
        return Quat.mul(a:conjugate(), b):normalize():to_axis_angle()
    end
    
    return Quat
end
files['modules/param_table'] = function(...)
    
    
    local PT = {}
    
    
    
    function PT.new(key, prefix, l)
      local self = {}
      local _key = key
      local _prefix = prefix
      local _l = l
      local _count = 0
      assert(param:add_table(_key, _prefix, _l), 'could not add param table')
    
      function self:param(name, default_value)
          assert(
            param:add_param(_key, _count + 1, name, default_value),
            string.format('could not add param %s', name)
          )
          _count = _count + 1
          return Parameter(_prefix .. name)
      end
      return self
    end
    
    
    function PT.get_value(p_or_v)
      if type(p_or_v) == 'number' then
        return p_or_v
      else
        return p_or_v:get()
      end
    end
    
    
    return PT
    

end
files['modules/controllers/pid'] = function(...)
    local PT = require('modules/param_table')
    local PID = {}
    
    -- constrain a value between limits
    function PID.constrain(v, vmin, vmax)
        if v < vmin then
           v = vmin
        end
        if v > vmax then
           v = vmax
        end
        return v
    end
    
     
    function PID.new(name, kFF, kP,kI,kD,min,max)
       local self = {}
       local _name = name
    
       local _kFF = kFF
       local _kP = kP
       local _kI = kI
       local _kD = kD
       local _min = min
       local _max = max
       
       function self:kFF()
          return PT.get_value(_kFF)
       end
       function self:kP()
          return PT.get_value(_kP)
       end
       function self:kI()
          return PT.get_value(_kI)
       end
       function self:kD()
          return PT.get_value(_kD)
       end
       function self:min()
          return PT.get_value(_min)
       end
       function self:max()
          return PT.get_value(_max)
       end
    
       local _FF = 0
       local _P = 0
       local _I = (self:min() + self:max()) / 2
       local _D = 0
    
       local _t = nil
       local _err = nil
       local _total = _I
    
       function self:update(ff, target, current)
          local t = millis():tofloat() * 0.001
          if not _t then
             _t = t
          end
          local err = target - current
          local dt = t - _t
          local mi = self:min()
          local ma = self:max()
          _FF = self:kFF() * ff
          _P = self:kP() * err
          if ((_total < ma and _total > mi) or (_total >= ma and err < 0) or (_total <= mi and err > 0)) then
             _I = PID.constrain(_I + self:kI() * err * dt, mi, ma)
          end
          if dt > 0 then
             _D = self:kD() * (err - _err) / dt
          end
          
          _t = t
          _err = err
          _total = PID.constrain(_FF + _P + _I + _D, mi, ma)
          
          logger.write(
             _name,'ff,Targ,Curr,err,dt,FF,P,I,D,Total','ffffffffff',
             ff,target,current,err,dt,_FF, _P,_I,_D,_total
          )
    
          gcs:send_custom_pid_state(_name, current, target, dt, _FF, _P,_I,_D)
    
          return _total
       end
    
       function self:reset(value)
          if value == nil then
             value = (self:min() + self:max()) / 2
          end
          _I = value
          _t=nil
          _err = nil
          _total = value
       end
    
       return self
    end
    
    return PID
end
files['modules/state'] = function(...)
    
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
        function self:pitch_angle()
            return _pitch
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
end
local flightmodes = require('modules/mappings/modes')
local State = require('modules/state')

function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local state = State.readCurrent()
        gcs:send_named_float("st_arspd", state:arspd())
        gcs:send_named_float("st_flow", state:flow():length())
        gcs:send_named_float("st_w_spd", state:wind():length())
    end
    return update, 250
end

return update()