

local Point = {}

function Point.new(p)
    local self = {}
    local _p = p

    function self.x()
        return _p:x()
    end
    function self.y()
        return _p:y()
    end
    function self.z()
        return _p:z()
    end
    function self.copy()
        return Point.xyz(self:x(), self:y(), self:z())
    end
    function self.p()
        return _p
    end

    function self.length()
        return _p:length()
    end

    function self.scale(f)
        return Point.xyz(_p:x()*f, _p:y()*f, _p:z()*f)
    end

    function self.unit()
        return Point.new(self.scale(1/self.length()))
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
    return Point.xyz(p1:y()*p2:z() - p1:z()*p2:y(), p1:z()*p2:x() - p1:x()*p2:z(), p1:x()*p2:y() - p1:y()*p2:x())
end


return Point