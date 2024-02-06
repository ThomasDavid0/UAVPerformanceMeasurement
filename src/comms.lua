

local comms = {}

function comms.gcsWrite(text)
    gcs:send_text(6, string.format("LUA: %s", text))
end


return comms
