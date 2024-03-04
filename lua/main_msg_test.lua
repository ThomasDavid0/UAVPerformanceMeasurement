
local curr = 0
local targ = 0
function update()
    curr = curr + 1
    targ = targ + 1
    gcs:send_text(6, 'sending pid state')
    gcs:send_custom_pid_state('TSPD', curr, -targ)
    return update, 1000
end


return update()


