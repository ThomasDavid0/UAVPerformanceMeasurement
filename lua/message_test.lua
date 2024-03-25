
function update()
    local state = State.readCurrent()

    GCS:send_float('pitch_angle', math.deg(state:pitch_angle()))
    GCS:send_float('pitch_angle', math.deg(state:roll_angle()))
    
    return update, 1000.0/40
end


return update()
