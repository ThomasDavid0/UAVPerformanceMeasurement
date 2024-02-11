
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



function PID.new(name,kP,kI,kD,min,max)
   local self = {}
   local _name = name
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
   
   local _min = min
   local _max = max
   
   local _P = 0
   local _I = (_min + _max) / 2
   local _D = 0

   local _t = nil
   local _err = nil
   local _total = (_min + _max) / 2

   function self:update(target, current)
      local t = millis():tofloat() * 0.001
      if not _t then
         _t = t
      end
      local err = target - current
      local dt = t - _t

      _P = _kP * err
      if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
         _I = PID.constrain(_I + _kI * err * dt, _min, _max)
      end
      if dt > 0 then
         _D = _kD * (err - _err) / dt
      end
      
      _t = t
      _err = err
      _total = PID.constrain(_P + _I + _D, _min, _max)
      
      logger.write(
         _name,'Targ,Curr,err,dt,P,I,D,Total','ffffffff',
         target,current,err,dt,_P,_I,_D,_total
      )

      return _total
   end

   function self:reset()
      _I = (_min + _max) / 2
      _t=nil
      _err = nil
      _total = (_min + _max) / 2
   end

   function self:set_I(I)
      _kI = I
   end

   function self:set_P(P)
      _kP = P
   end

   function self:set_D(D)
      _kD = D
   end
   
   
   -- return the instance
   return self
end

return PID