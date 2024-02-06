local g = {}

function g.makeVector3f(x, y, z)
    local vec = Vector3f()
    vec:x(x)
    vec:y(y)
    vec:z(z)
    return vec
end

function g.vec_mul(v, s)
    return g.makeVector3f(v:x() * s, v:y() * s, v:z() * s)
end

function g.makeQuaternion(w, x, y, z)
    local q = Quaternion()
    q:q1(w)
    q:q2(x)
    q:q3(y)  
    q:q4(z)
    return q
end

function g.quat_log(name, q)
    logger.write(
        name, 'w,x,y,z', 'ffff',
        q:q1(), q:q2(), q:q3(), q:q4()
    )
end

function g.qorient(roll, pitch, yaw)
    local q = Quaternion()
    q:from_euler(roll, pitch, yaw)
    return q
end


function g.qorient_deg(roll_deg, pitch_deg, yaw_deg)
    local q = Quaternion()
    q:from_euler(math.rad(roll_deg), math.rad(pitch_deg), math.rad(yaw_deg))
    return q
end

function g.quat_earth_to_body(quat, v)
    local v2 = v:copy()
    quat:earth_to_body(v2)
    return v2
end

function g.quat_body_to_earth(quat, v)
    local v2 = v:copy()
    quat:inverse():earth_to_body(v2)
    return v2
end

function g.quat_copy(q)
    return q:inverse():inverse()
end

function g.angle_between(v1, v2)
    return v1:angle(v2)
end


function g.quat_axis(q)
    local q2 = g.quat_copy(q)
    return g.makeVector3f(q2:q2(),q2:q3(),q2:q4())
end

function g.quat_conjugate(q)
    local q2 = g.quat_copy(q)
    return g.makeQuaternion(q2:q1(), -q2:q2(), -q2:q3(), -q2:q4())
end

function g.quat_mul(a, b)
    local w = a:q1() * b:q1() - g.quat_axis(a):dot(g.quat_axis(b))
    local xyz = g.vec_mul(g.quat_axis(b), a:q1())  + g.vec_mul(g.quat_axis(a), b:q1()) + g.quat_axis(a):cross(g.quat_axis(b))
    return g.makeQuaternion(w, xyz:x(), xyz:y(), xyz:z())
end

function g.body_axis_rates(a,b)
    local q = g.quat_mul(g.quat_conjugate(a), b)
    q:normalize()
    local vo = Vector3f()
    q:to_axis_angle(vo)
    return vo
end

--[[
   get quaternion rotation between vector1 and vector2
   with thanks to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
--]]
function g.vectors_to_quat_rotation(vector1, vector2)
    local v1 = vector1:copy()
    local v2 = vector2:copy()
    v1:normalize()
    v2:normalize()
    local dot = v1:dot(v2)
    local a = v1:cross(v2)
    local w = 1.0 + dot
    local q = Quaternion()
    q:q1(w)
    q:q2(a:x())
    q:q3(a:y())
    q:q4(a:z())
    q:normalize()
    return q
 end

 -- convert a quaternion to axis angle form
function g.to_axis_and_angle(quat)
    local axis_angle = Vector3f()
    quat:to_axis_angle(axis_angle)
    local angle = axis_angle:length()
    if angle < 0.00001 then
       return g.makeVector3f(1.0, 0.0, 0.0), 0.0
    end
    return axis_angle:scale(1.0/angle), angle
 end


 return g