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