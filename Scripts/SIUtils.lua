--[[
***************************************************************************

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    Author: Jacopo De Stefani <jacopo.de.stefani@gmail.com>

***************************************************************************
]]

--[[  File containing different utility functions for the project
  ]]


--[[ Function used to copy a table in order to prevent modifications
	  of the internal state of the robot ]]
function CopyTable(t)
   local t2 = {}
   for key,value in pairs(t) do
      t2[key] = value
   end
   return t2
end

function PrintTable(t)
   for key,value in pairs(t) do
      log(key .. " = " .. value .. ",")
   end
end

--[[ Function used to convert the proximity sensor index to the corrisponding angle
	 Beware: Approximative calculation ]]
function IndexToRadians(index)
   return (math.pi/#robot.proximity)*index
end


--[[ Function to get the id of a beacon in the sensedBeacons table, given its ID ]]
function FindBeaconFromId(id,sensedBeacons)
   for key,value in pairs(sensedBeacons) do
      if value.id == id then
		return key
	  end
   end
   return nil
end

