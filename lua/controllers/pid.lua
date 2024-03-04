
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

   local _kFF = kFF or 0.0
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
   
   local _min = min
   local _max = max
   
   local _FF = 0
   local _P = 0
   local _I = (_min + _max) / 2
   local _D = 0

   local _t = nil
   local _err = nil
   local _total = (_min + _max) / 2

   function self:update(ff, target, current)
      local t = millis():tofloat() * 0.001
      if not _t then
         _t = t
      end
      local err = target - current
      local dt = t - _t
      
      _FF = _kFF * ff
      _P = _kP * err
      if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
         _I = PID.constrain(_I + _kI * err * dt, _min, _max)
      end
      if dt > 0 then
         _D = _kD * (err - _err) / dt
      end
      
      _t = t
      _err = err
      _total = PID.constrain(_FF + _P + _I + _D, _min, _max)
      
      logger.write(
         _name,'ff,Targ,Curr,err,dt,FF,P,I,D,Total','ffffffffff',
         ff,target,current,err,dt,_FF, _P,_I,_D,_total
      )

      gcs:send_custom_pid_state(_name, current, target, dt, _FF, _P,_I,_D)

      return _total
   end

   function self:reset(value)
      if value == nil then
         value = (_min + _max) / 2
      end
      _I = value
      _t=nil
      _err = nil
      _total = value
   end
   function self:I()
      return _I
   end
   function self:P()
      return _P
   end
   function self:D()
      return _D
   end
   function self:total()
      return _total
   end
   
   -- return the instance
   return self
end

return PID