

local PT = {}


function PT:new(key, prefix, l)
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
      return Parameter(_prefix .. name):get()
  end
  return self
end


--[[
  // @Param: AEROM_ANG_ACCEL
  // @DisplayName: Angular acceleration limit
  // @Description: Maximum angular acceleration in maneuvers
  // @Units: deg/s/s
--]]
--AEROM_ANG_ACCEL = bind_add_param('ANG_ACCEL', 1, 6000)

