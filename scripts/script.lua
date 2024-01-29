local MODE_AUTO = 10

local last_id = nil
local active_command = nil
local iterations = 0


function initialise(id, cmd)
    gcs:send_text(6, string.format("LUA: initialise command: %i, id: %i", cmd, id))
    last_id = id
    active_command = cmd
    iterations = 0
end


function timout()
    if active_command then
        gcs:send_text(6, string.format("LUA: Completed %i iterations", iterations))
        active_command = nil
    end
end


function do_something()
    iterations = iterations + 1
end


function update()

    if vehicle:get_mode() == MODE_AUTO then

        id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if id~=last_id then
                initialise(id, cmd)
            end
        else
            timout()
        end
        
        if active_command == 1 then
            do_something()
        end

    end

    return update, 1000.0/40
end


return update()