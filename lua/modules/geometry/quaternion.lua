
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