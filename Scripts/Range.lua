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
    Lua implementation of ARGoS CRange class by Carlo Pinciroli cpinciro@ulb.ac.be

***************************************************************************
]]

-- A class is created by means of a table with the same name as the class itself
Range = {}
-- And a metatable
Rangemt = {}

-- By adding the appropriate elements to the metatable it is possibile to override the default behavior of the operators
Rangemt.__index = Range -- redirect queries to the String table

--[[Constructor from range bounds, returns (0,0) if there is a mismatch in the values]]
function Range:new(lower_bound,upper_bound)
	if(lower_bound > upper_bound) then
		return setmetatable({ lowerBound = 0, upperBound = 0}, Rangemt)
	else
		return setmetatable({ lowerBound = lower_bound, upperBound = upper_bound}, Rangemt)
	end
end

function Range:ToString()
	return "[" .. self.lowerBound .. "," .. self.upperBound .. "]"
end

--[[ Function to test the inclusion in the range [lb,ub] ]]
function Range:WithinBoundsIncluded(value) 
         return (self.lowerBound <= value and value <= self.upperBound)
end

--[[ Function to test the inclusion in the range (lb,ub) ]]
function Range:WithinBoundsExcluded(value) 
         return (self.lowerBound < value and value < self.upperBound)
end

--[[ Function to test the inclusion in the range (lb,ub] ]]
function Range:WithinLowerBoundExcluded(value) 
         return (self.lowerBound < value and value <= self.upperBound)
end

--[[ Function to test the inclusion in the range [lb,ub) ]]
function Range:WithinUpperBoundExcluded(value) 
         return (self.lowerBound <= value and value <= self.upperBound)
end

--[[ Function to normalize a given value in the range ]]
function Range:NormalizeInRange(value) 
         return (value-self.lowerBound)/(self.upperBound-self.lowerBound)
end

--[[ Function to compute the range span ]]      
function Range:GetSpan() 
         return self.upperBound-self.lowerBound
end

--[[ Function to obtain a random number in the range ]]      
function Range:UniformRandom() 
         return self.lowerBound+(self.upperBound-self.lowerBound)*robot.random.uniform()
end


