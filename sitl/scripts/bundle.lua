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
       local _total = (self:min() + self:max()) / 2
    
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
    
        local _body_wind = _att:transform_point(_wind)
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
files['modules/mappings/functions'] = function(...)
    local functions = {
        aileron = 4,
        elevator = 19,
        throttle = 70,
        throttle_left = 73,
        throttle_right = 74,
        rudder = 21,
        flap = 2,
        automatic_flaps = 3,
        flaperon_left = 24,
        flaperon_right = 25,
        elevon_left = 77,
        elevon_right = 78,
        v_tail_left = 79,
        v_tail_right = 80,
        differential_spoiler_left1 = 16,
        differential_spoiler_right1 = 17,
        differential_spoiler_left2 = 86,
        differential_spoiler_right2 = 87,
        ground_steering = 26,
        boost_engine_throttle = 81,
        motor_enable_switch = 30,
        landing_gear = 29,
        air_brakes = 110,
    
    }
    return functions
end
files['modules/square_wave'] = function(...)
    
    
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
end
files['modules/controllers/pitch_angle_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(73, "SCRTPAN_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2.5)
    local kI = pt:param('kI', 0.3)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -90)
    local max = pt:param('max', 90)
    
    
    local controller = PID.new('TPAN', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['experiments/pitch_angle_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    local pitch_controller = require('modules/controllers/pitch_angle_controller')
    
    local sq = SQ.new(2, 5000, 5)
    
    local Exp = {}
    
    function Exp.setup()
        local self = {}
        local state = State.readCurrent()
    
        function self:run()
    
            vehicle:set_target_throttle_rate_rpy(
                    SRV_Channels:get_output_scaled(functions.throttle),
                    0, 
                    pitch_controller:update(0.0, sq:value(), math.deg(state:pitch_angle())),
                    0
                )
            vehicle:set_rudder_offset(0, true)
        end
    
        function self:reset()
            sq:reset()
            pitch_controller:reset()
        end
        return self
    end
    
    return Exp
end
files['modules/controllers/pa_alt_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(74, "SCRTPALT_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2.0)
    local kI = pt:param('kI', 0.0)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -40)
    local max = pt:param('max', 40)
    
    
    local controller = PID.new('TPAL', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['modules/controllers/alt_controller'] = function(...)
    
    local pitch_controller = require('modules/controllers/pitch_angle_controller')
    local pa_alt_controller = require('modules/controllers/pa_alt_controller')
    
    local cnt = {}
    
    
    function cnt:update(ff, target_alt, current_alt, current_pitch)
        local pitch_target = pa_alt_controller:update(ff, target_alt, current_alt)
    
        return pitch_controller:update(0, pitch_target, current_pitch)
    end
    
    function cnt:reset()
        pitch_controller:reset()
        pa_alt_controller:reset()
    end
    
    
    return cnt
end
files['experiments/alt_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local alt_controller = require('modules/controllers/alt_controller')
    
    local sq = SQ.new(100, 10000, 5)
    local PCCT = {}
    
    function PCCT.setup()
        local self = {}
        local state = State.readCurrent()
    
        function self:run()
            local target = sq:value()
    
            local pitch = alt_controller:update(0.0, target, -state:pos():z(), math.deg(state:pitch_angle()))
    
            vehicle:set_target_throttle_rate_rpy(
                SRV_Channels:get_output_scaled(functions.throttle),
                0, pitch, 0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        function self:reset()
            sq:reset()
            alt_controller:reset()
        end
    
        return self
    end
    
    
    
    return PCCT
end
files['modules/controllers/speed_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(71, "SCRTSPD_", 6)
    
    local kFF = pt:param('kFF', 1)
    local kP = pt:param('kP', 60)
    local kI = pt:param('kI', 50)
    local kD = pt:param('kD', 0)
    local min = pt:param('min', 0)
    local max = pt:param('max', 100)
    
    
    local speed_controller = PID.new('TSPD', kFF, kP, kI, kD, min, max)
    
    return speed_controller
end
files['experiments/speed_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    local speed_controller = require('modules/controllers/speed_controller')
    local alt_controller = require('modules/controllers/alt_controller')
    
    local sq = SQ.new(24, 10000, 4)
    
    local SCT = {}
    function SCT.setup()
        local self = {}
        local state = State.readCurrent()
    
        function self:run()
            
            local pitch = alt_controller:update(0.0, 100, -state:pos():z(), math.deg(state:pitch_angle()))
    
            local target = sq:value()
            local thr = speed_controller:update(0.0, target, state:flow():length())
    
            vehicle:set_target_throttle_rate_rpy(thr, 0, pitch, 0)
            vehicle:set_rudder_offset(0, true)
        end
    
        function self:reset()
            sq:reset()
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            alt_controller:reset()
        end
    
        return self
    end
    
    return SCT
    
    

end
files['modules/controllers/roll_angle_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(72, "SCRTRAN_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 20)
    local kI = pt:param('kI', 0.2)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -90)
    local max = pt:param('max', 90)
    
    
    local controller = PID.new('TRAN', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['experiments/roll_angle_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    local roll_controller = require('modules/controllers/roll_angle_controller')
    local speed_controller = require('modules/controllers/speed_controller')
    local alt_controller = require('modules/controllers/alt_controller')
    
    
    local sq = SQ.new(0, 5000, 10)
    local RCCT = {}
    
    function RCCT.setup()
        local self = {}
        local state = State.readCurrent()
    
        function self:run()
            local target = sq:value()
    
            local roll = roll_controller:update(0.0,target, math.deg(state:roll_angle()))
            local pitch = alt_controller:update(0.0, 100, -state:pos():z(), math.deg(state:pitch_angle()))
            local thr = speed_controller:update(0.0, 22, state:flow():length())
    
            vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, 0)
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
    
    
    
    return RCCT
end
local flightmodes = require('modules/mappings/modes')

local experiments = {}
experiments[0] = require('experiments/pitch_angle_controller_tuning')
experiments[1] = require('experiments/alt_controller_tuning')
experiments[2] = require('experiments/speed_controller_tuning')
experiments[3] = require('experiments/roll_angle_controller_tuning')

function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()

        local exp = experiments[0].setup()

        if cmd then
            exp:run()
        else
            if exp then
                exp:reset()
            end
        end
    end
    return update, 1000.0/40
end


return update()


