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

-- A class is created by means of a table with the same name as the class itself
Vector2 = {}
-- And a metatable
Vector2mt = {}

-- By adding the appropriate elements to the metatable it is possibile to override the default behavior of the operators
-- Operator overriding : + 
Vector2mt.__add = function (a,b) return Vector2:fromXY(a.x+b.x,a.y+b.y) end
-- Operator overriding : Binary -
Vector2mt.__sub = function (a,b) return Vector2:fromXY(a.x-b.x,a.y-b.y) end
-- Operator overriding : Unary -
Vector2mt.__unm = function (a) return Vector2:fromXY(-a.x,-a.y) end
-- Operator overriding : ==
Vector2mt.__eq = function (a,b) return a.x == b.x and a.y == b.y end
Vector2mt.__index = Vector2 -- redirect queries to the String table

--[[ Constructor from polar coordinates ]]
function Vector2:fromPolar(length,angle)
	return setmetatable({ x = length * math.cos(angle) , y = length * math.sin(angle) }, Vector2mt)
end

--[[ Constructor from XY coordinates ]]
function Vector2:fromXY(x_in,y_in)
	return setmetatable({ x=x_in , y=y_in}, Vector2mt)
end

function Vector2:toString()
	return "[x=" .. self.x .. ",y=" .. self.y .. "]"
end

function Vector2:toStringPolar()
	return "[l=" .. Vector2.length(self) .. ",a=" .. Vector2.angle(self) .. "]"
end

--[[ Constructor from polar coordinates ]]
function Vector2:SetFromPolar(length,angle)
	self.x = length * math.cos(angle) 
	self.y = length * math.sin(angle)
end

--[[ Constructor from XY coordinates ]]
function Vector2:SetFromXY(x_in,y_in)
	self.x=x_in
	self.y=y_in
end

--[[ Function used to compute the angle from a 2D vector ]]
function Vector2:angle()
	return math.atan2(self.y, self.x)
end

--[[ Function used to compute the length of 2D vector ]]
function Vector2:length()
	return math.sqrt(math.pow(self.x,2) + math.pow(self.y,2))
end

--[[ Function used to obtain the representation of the current vector in polar coordinates ]]
function Vector2:polarVector()
   local polar = {length=self.length(),angle=self.angle()}
   return polar
end

--[[ Function used to rotate the vector:
	 # angle > 0 : Counterclockwise
	 # angle < 0 : Clockwise
  ]]
function Vector2:Rotate(angle)
   self.x = math.cos(angle)*self.x - math.sin(angle)*self.y
   self.y = math.sin(angle)*self.x + math.cos(angle)*self.y
end

--[[ Function used to rescale the current vector ]]
function Vector2:Scale(k)
   self.x = k*self.x 
   self.y = k*self.y 
end
