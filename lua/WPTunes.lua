local RUN_INTERVAL_MS = 500

-- get number of waypoints, excluding home
local last_mission_length = mission:num_commands() - 1
local last_nav_wp = mission:get_current_nav_index()

-- set tunes for mission changes
local function cleared()
    notify:play_tune('MFT180MLO1L16AC<A')
end

local function added()
    notify:play_tune('MFT180MLO1L16A>CA')
end

local function removed()
    notify:play_tune('MFT180MLO2L16AC<A')
end

local function new_wp()
    notify:play_tune('MFT180MSO2L16G0G')
end

function tunes()

    local mission_length = mission:num_commands() - 1
    local current_wp = mission:get_current_nav_index()

    if mission_length ~= last_mission_length then
        gcs:send_text(0, string.format("Mission: new num commands: %d",mission_length))
        -- mission cleared
        if mission_length < 1 then
            cleared()
        end
    
        -- new waypoint added
        if mission_length > last_mission_length then
            added()
        end
        
        -- waypoint removed
        if mission_length < last_mission_length and mission_length > 0 then
            removed()
        end
        
        last_mission_length = mission_length
    end
    
    -- current WP has changed
    if current_wp ~= last_nav_wp then
        new_wp()
        last_nav_wp = current_wp
    end
    
    return tunes, RUN_INTERVAL_MS
end

return tunes, RUN_INTERVAL_MS
