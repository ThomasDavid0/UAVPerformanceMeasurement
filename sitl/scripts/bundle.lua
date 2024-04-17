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
    
       function self:I()
          return _I
       end
    
       function self:total()
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
files['modules/controllers/c71_speed_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(71, "SCRTSPD_", 6)
    
    local kFF = pt:param('kFF', 1.2)
    local kP = pt:param('kP', 5.0)
    local kI = pt:param('kI', 1.0)
    local kD = pt:param('kD', 0)
    local min = pt:param('min', 0)
    local max = pt:param('max', 100)
    
    
    local speed_controller = PID.new('TSPD', kFF, kP, kI, kD, min, max)
    
    return speed_controller
end
files['modules/controllers/c72_roll_angle_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(72, "SCRTRAN_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 5.0)
    local kI = pt:param('kI', 1.0)
    local kD = pt:param('kD', 2.0)
    local min = pt:param('min', -90)
    local max = pt:param('max', 90)
    
    
    local controller = PID.new('TRAN', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['modules/controllers/c73_pitch_angle_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(73, "SCRTPAN_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2)
    local kI = pt:param('kI', 0.3)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -90)
    local max = pt:param('max', 90)
    
    
    local controller = PID.new('TPAN', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['modules/controllers/c74_pitch_alt_controller'] = function(...)
    
    local pitch_controller = require('modules/controllers/c73_pitch_angle_controller')
    
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(74, "SCRTPALT_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2.2)
    local kI = pt:param('kI', 0.1)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -30)
    local max = pt:param('max', 30)
    
    local controller = PID.new('TPAL', kFF, kP, kI, kD, min, max)
    
    local cnt = {}
    
    function cnt:update(target_alt, current_alt, current_roll, current_pitch)
        local pitch_target = controller:update(
            -1 / math.cos(math.rad(current_roll)),
            target_alt, current_alt
        )
        return pitch_controller:update(0, pitch_target, current_pitch)
    end
    
    function cnt:reset(value)
        pitch_controller:reset()
        controller:reset()
    end
    
    function cnt:angle_controller()
        return controller
    end
    
    function cnt:pitch_controller()
        return pitch_controller
    end
    
    
    return cnt
end
files['modules/controllers/c75_yaw_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(75, "SCRTYAW_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2.0)
    local kI = pt:param('kI', 5.0)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', -90)
    local max = pt:param('max', 90)
    
    
    local controller = PID.new('TYAW', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['modules/controllers/c76_pitch_g_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(76, "SCRTPG_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 2)
    local kI = pt:param('kI', 12)
    local kD = pt:param('kD', 2)
    local min = pt:param('min', -180)
    local max = pt:param('max', 180)
    
    
    local controller = PID.new('TPG', kFF, kP, kI, kD, min, max)
    
    return controller
end
files['modules/controllers/c77_turn_climb_controller'] = function(...)
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(77, "SCRTRCLB_", 6)
    
    local kFF = pt:param('kFF', 0)
    local kP = pt:param('kP', 0.1)
    local kI = pt:param('kI', 0.1)
    local kD = pt:param('kD', 0.0)
    local min = pt:param('min', 5)
    local max = pt:param('max', 85)
    
    local controller = PID.new('TRCL', kFF, kP, kI, kD, min, max)
    
    local cnt = {}
    
    function cnt:update(target_climb_rate, current_climb_rate, current_roll_angle)
        local target_roll_angle = controller:update(0, target_climb_rate, current_climb_rate)
        return roll_controller:update(0, target_roll_angle, current_roll_angle)
    end
    
    function cnt:reset(value)
        roll_controller:reset()
        controller:reset()
    end
    
    
    return cnt
end
files['modules/controllers/c78_thr_acc_controller'] = function(...)
    local PID = require('modules/controllers/pid')
    local PT = require('modules/param_table')
    
    local pt = PT.new(78, "SCRTSPA_", 6)
    
    local kFF = pt:param('kFF', 1.2)
    local kP = pt:param('kP', 5.0)
    local kI = pt:param('kI', 0.0)
    local kD = pt:param('kD', 0)
    local min = pt:param('min', 0)
    local max = pt:param('max', 100)
    
    
    local speed_controller = PID.new('TSPA', kFF, kP, kI, kD, min, max)
    
    return speed_controller
end
files['modules/controllers/controllers'] = function(...)
    local controllers = {}
    
    controllers.TSPD = require('modules/controllers/c71_speed_controller')
    controllers.TRAN = require('modules/controllers/c72_roll_angle_controller')
    controllers.TPAN = require('modules/controllers/c73_pitch_angle_controller')
    controllers.TPALT = require('modules/controllers/c74_pitch_alt_controller')
    controllers.TYAW = require('modules/controllers/c75_yaw_controller')
    controllers.TPG = require('modules/controllers/c76_pitch_g_controller')
    controllers.TRCLB = require('modules/controllers/c77_turn_climb_controller')
    controllers.TSPA = require('modules/controllers/c78_thr_acc_controller')
    
    
    function controllers.reset_all(defaults)
        controllers.TSPD:reset(defaults.TSPD)
        controllers.TRAN:reset(defaults.TRAN)
        controllers.TPAN:reset(defaults.TPAN)
        controllers.TPALT:reset(defaults.TPALT)
        controllers.TYAW:reset(defaults.TYAW)
        controllers.TPG:reset(defaults.TPG)
        controllers.TRCLB:reset(defaults.TRCLB)
        controllers.TSPA:reset(defaults.TSPA)
    end
    
    return controllers
end
files['experiments/exp0_pitch_angle_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local controllers = require('modules/controllers/controllers')
    --local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    --local pitch_controller = require('modules/controllers/c73_pitch_angle_controller')
    --local speed_controller = require('modules/controllers/c71_speed_controller')
    local sq = SQ.new(2, 6000, 10)
    
    local istate
    
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq:reset()
            controllers.reset_all(
                {TSPD=SRV_Channels:get_output_scaled(functions.throttle)}
            )
            --pitch_controller:reset()
            --roll_controller:reset()
            --speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            istate = state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                controllers.TSPD:update(
                    state:pitch_angle_deg(),
                    istate:arspd(),
                    state:arspd()
                ),
                controllers.TRAN:update(0.0, 0, state:roll_angle_deg()),
                controllers.TPAN:update(0.0, sq:value(), state:pitch_angle_deg()),
                0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
end
files['experiments/exp1_alt_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local sq
    
    local istate
    
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq = SQ.new(-state:pos():z(), 8000, 5)
            alt_controller:reset()
            roll_controller:reset()
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            istate = state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(
                    state:pitch_angle_deg(),
                    istate:arspd(),
                    state:arspd()
                ),
                roll_controller:update(0.0, 0, state:roll_angle_deg()),
                alt_controller:update(
                    sq:value(),
                    -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ),
                0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
end
files['experiments/exp2_roll_angle_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local sq = SQ.new(0, 5000, 30)
    
    local Exp = {}
    local istate
    
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq:reset()
            roll_controller:reset()
            alt_controller:reset()
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            istate = state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, istate:arspd(), state:arspd()),
                roll_controller:update(0.0, sq:value(), state:roll_angle_deg()),
                alt_controller:update(
                    -istate:pos():z(), -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ),
                0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    
    
    return Exp
end
files['experiments/exp3_speed_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local sq
    
    local istate
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq = SQ.new(state:arspd() + 2, 10000, 4)
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            alt_controller:reset()
            roll_controller:reset()
            istate=state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(
                    state:pitch_angle_deg(),
                    sq:value(),
                    state:arspd()
                ),
                roll_controller:update(0.0, 0, state:roll_angle_deg()),
                alt_controller:update(
                    -istate:pos():z(),
                    -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ), 0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
    
    

end
files['experiments/exp4_yaw_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local yaw_controller = require('modules/controllers/c75_yaw_controller')
    local sq = SQ.new(0, 5000, 2)
    
    local Exp = {}
    local initial_state
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq:reset()
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            alt_controller:reset()
            roll_controller:reset()
            yaw_controller:reset()
            initial_state = state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, initial_state:arspd(), state:arspd()),
                roll_controller:update(0.0, 0, state:roll_angle_deg()),
                alt_controller:update(
                    -initial_state:pos():z(),
                    -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ),
                yaw_controller:update(0.0, sq:value(), -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
    
    

end
files['experiments/exp5_pitch_g_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local pitch_g_controller = require('modules/controllers/c76_pitch_g_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local sq = SQ.new(1, 3000, 1.0)
    
    local istate
    local start_t
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq:reset()
            pitch_g_controller:reset()
            roll_controller:reset()
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            start_t = millis()
            istate = state
        end
    
        function self:run()
            if (millis() - start_t) < 7000 then
                vehicle:set_target_throttle_rate_rpy(
                    speed_controller:update(
                        state:pitch_angle_deg(),
                        istate:arspd(),
                        state:arspd()
                    ),
                    roll_controller:update(0.0, 0, state:roll_angle_deg()),
                    pitch_g_controller:update(0, sq:value(), -state:acc():z()/9.81),
                    0
                )
                vehicle:set_rudder_offset(0, true)
            end
        end
    
        return self
    end
    
    return Exp
end
files['modules/turn'] = function(...)
    
    
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
        function self:roll_angle_deg()
            return math.deg(_roll_angle)
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
    
        return self
    end
    
    
    function Turn.timout(active_turn)
        if active_turn then
            gcs:send_text(6, string.format("timeout %s",active_turn.summary()))
            return nil
        end
    end
    
    
    return Turn
end
files['experiments/exp6_coordinated_turn'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    local Turn = require('modules/turn')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local yaw_controller = require('modules/controllers/c75_yaw_controller')
    
    local turn
    
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
        
        function self:reset()
            turn = Turn.new(0, 2, -state:pos():z(), state:arspd())
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            alt_controller:reset()
            roll_controller:reset()
            yaw_controller:reset()
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, turn:arspd(), state:arspd()),
                roll_controller:update(0, turn:roll_angle_deg(), state:roll_angle_deg()),
                alt_controller:update(
                    turn:alt(),
                    -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ),
                yaw_controller:update(0.0, 0.0, -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
    
    

end
files['experiments/exp7_gturn'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local Turn = require('modules/turn')
    
    local speed_controller = require('modules/controllers/c71_speed_controller')
    local pitch_alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    local yaw_controller = require('modules/controllers/c75_yaw_controller')
    local turn_climb_controller = require('modules/controllers/c77_turn_climb_controller')
    local pitch_g_controller = require('modules/controllers/c76_pitch_g_controller')
    local turn = nil
    
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:run()
            if turn then
                local _roll = 0
                local _pitch = 0
    
                if turn:stage() == 0 then
                    if state:roll_angle() > turn:roll_angle() then
                        turn:next_stage()
                        pitch_g_controller:reset(pitch_alt_controller:pitch_controller():total())
                        
                    else
                        _roll = 10
                        _pitch = pitch_alt_controller:update(
                            turn:alt(),
                            -state:pos():z(),
                            state:roll_angle_deg(),
                            state:pitch_angle_deg()
                        )
                    end
                end
                if turn:stage() == 1 then
                    _roll = turn_climb_controller:update(
                        0, state:att():transform_point(state:vel()):z(),
                        state:roll_angle_deg()
                    )
                    _pitch = pitch_g_controller:update(0, turn:load_factor(), -state:acc():z()/9.81)
                end
    
                vehicle:set_target_throttle_rate_rpy(
                    speed_controller:update(0.0, turn:arspd(), state:arspd()),
                    _roll, _pitch,
                    yaw_controller:update(0.0, 0.0, -state:flow():y())
                )
                vehicle:set_rudder_offset(0, true)
    
            else
                turn = Turn.new(
                    0,
                    2, --1.2 + 0.4 * ((_id-1) % 5),
                    -state:pos():z(),
                    state:arspd() --15 + 3 * math.floor((id-1) / 5)
                )
            end
        end
    
        function self:reset()
            turn = nil
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            pitch_alt_controller:reset()
            pitch_g_controller:reset()
            turn_climb_controller:reset()
            yaw_controller:reset()
        end
    
        return self
    end
    
    return Exp
    
    

end
files['experiments/exp8_speed_acc_controller_tuning'] = function(...)
    
    local State = require('modules/state')
    local functions = require('modules/mappings/functions')
    local SQ = require('modules/square_wave')
    
    local roll_controller = require('modules/controllers/c72_roll_angle_controller')
    local acc_controller = require('modules/controllers/c78_thr_acc_controller')
    local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
    
    local sq
    local istate
    local Exp = {}
    function Exp.setup(id, cmd)
        local self = {}
        local state = State.readCurrent()
    
        function self:reset()
            sq = SQ.new(0, 6000, 1)
            acc_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
            alt_controller:reset()
            roll_controller:reset()
            istate=state
        end
    
        function self:run()
            vehicle:set_target_throttle_rate_rpy(
                acc_controller:update(
                    0,
                    sq:value(),
                    state:acc():x()
                ),
                roll_controller:update(0.0, 0, state:roll_angle_deg()),
                alt_controller:update(
                    -istate:pos():z(),
                    -state:pos():z(),
                    state:roll_angle_deg(),
                    state:pitch_angle_deg()
                ), 0
            )
            vehicle:set_rudder_offset(0, true)
        end
    
        return self
    end
    
    return Exp
    
    

end
files['experiments/exp9_parabola'] = function(...)
    
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
    
    

end
local flightmodes = require('modules/mappings/modes')
local PT = require('modules/param_table')

local experiments = {}
experiments[0] = require('experiments/exp0_pitch_angle_controller_tuning')
experiments[1] = require('experiments/exp1_alt_controller_tuning')
experiments[2] = require('experiments/exp2_roll_angle_controller_tuning')
experiments[3] = require('experiments/exp3_speed_controller_tuning')
experiments[4] = require('experiments/exp4_yaw_controller_tuning')
experiments[5] = require('experiments/exp5_pitch_g_controller_tuning')
experiments[6] = require('experiments/exp6_coordinated_turn')
experiments[7] = require('experiments/exp7_gturn')
experiments[8] = require('experiments/exp8_speed_acc_controller_tuning')
experiments[9] = require('experiments/exp9_parabola')

local pt = PT.new(69, "SCRTEXP_", 7)

local expid = pt:param('EXPID', 0)
local running_exp = nil
function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        local eid = expid:get()
        local exp = experiments[eid].setup(id, cmd)

        if cmd then
            if running_exp == eid then
                logger:write('TEXP', 'id', 'i', eid )
                exp:run()
            end
        else
            if exp then
                exp:reset()
                running_exp = eid
            end
        end
    end
    return update, 1000.0/40
end


return update()
