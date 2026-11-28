-- plnd_probe.lua: test script, flies nothing. Copy to the Pixhawk SD card: APM/scripts/
-- Every 2 s it prints in the QGC messages:
--   memory used by Lua, whether precision landing sees the target (from the Pi LANDING_TARGET),
--   the lidar distance and the EKF relative position.
-- Checks that the Pixhawk 1 has enough memory and that the bindings the landing script
-- needs exist on this firmware. Remove it from the SD card after the test.

local MAV_INFO = 6
local ROT_DOWN = 25

local function try(f)
  local ok, v = pcall(f)
  if ok then return v end
  return "ERR"
end

local function fmt(v)
  if v == nil then return "nil" end
  if type(v) == "number" then return string.format("%.2f", v) end
  return tostring(v)
end

gcs:send_text(MAV_INFO, "PROBE: start")

local function update()
  local mem = collectgarbage("count")
  local acq = try(function() return precland:target_acquired() end)
  local hl = try(function() return precland:healthy() end)
  local rng = try(function() return rangefinder:distance_orient(ROT_DOWN) end)
  local pos = try(function() return ahrs:get_relative_position_NED_origin() end)
  local px, py, pz = "nil", "nil", "nil"
  if type(pos) == "userdata" then
    px, py, pz = fmt(pos:x()), fmt(pos:y()), fmt(pos:z())
  elseif pos == "ERR" then
    px = "ERR"
  end
  local tgt = try(function() return precland:get_target_location() end)
  local tgt_s = "nil"
  if type(tgt) == "userdata" then tgt_s = "ok" elseif tgt == "ERR" then tgt_s = "ERR" end
  gcs:send_text(MAV_INFO, string.format("PROBE mem %.0fkB plnd %s/%s tgt %s rng %s pos %s,%s,%s",
    mem, fmt(hl), fmt(acq), tgt_s, fmt(rng), px, py, pz))
  return update, 2000
end

return update, 2000
