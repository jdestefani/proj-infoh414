--[[  File containing different utility functions for the project
  ]]

--
--function vec2_new_polar(length, angle)
   --local vec2 = {
     -- x = length * math.cos(angle)
    --  y = length * math.sin(angle)
   --}
  -- return vec2
--end

-- Summing two 2D vectors (v1 = v1 + v2)
function vec2DSum(v1, v2)
   v1.x = v1.x + v2.x
   v1.y = v1.y + v2.y
end

--[[ Function used to compute the angle from a 2D vector ]]
function vec2DAngle(v)
   return math.atan2(v.y, v.x)
end

--[[ Function used to compute the length of 2D vector ]]
function vec2DLength(v)
   return math.sqrt(math.pow(v.x,2) + math.pow(v.y,2))
end

--[[ Function used to compute the angle from a 2D vector ]]
function vec2Polar(v)
   polar = {vec2Length,vec2Angle(v)}
	return polar
end

--[[ Function used to copy a table in order to prevent modifications
	  of the internal state of the robot ]]
function CopyTable(t)
   local t2 = {}
   for key,value in pairs(t) do
      t2[key] = value
   end
   return t2
end
