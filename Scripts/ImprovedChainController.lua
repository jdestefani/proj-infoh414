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

--Path should be given with respect to the execution path of ARGoS
dofile("./Scripts/Vector2.lua")
dofile("./Scripts/Range.lua")
dofile("./Scripts/SIUtils.lua")

-- Sensors: 
-- # Distance scanner
-- # Ground sensors
-- # Proximity sensors

-- Actuators:
-- # Wheels
-- # Range and bearing system

-- States: 
STATE = {
EXIT_NEST = {text="Exit nest", encoding=0},
SPOT_REACHED = {text="Spot reached", encoding=1},
CHAIN_END = {text="Chain end", encoding=2},
CHAIN_MEMBER = {text="Chain member", encoding=3},
CHAIN_JUNCTION = {text="Chain junction", encoding=4},
CHAIN_FOLLOWING = {text="Chain following", encoding=5},
ATTACHING = {text="Attaching", encoding=6}
}

-- Readings
POSITION = {
IN_SPOT = 0,
FRONT_IN_SPOT = 1,
BACK_IN_SPOT = 2,
LEFT_IN_SPOT = 3,
RIGHT_IN_SPOT = 4,
OUT_OF_NEST = 5,
FRONT_IN_NEST = 6,
BACK_IN_NEST = 7,
LEFT_IN_NEST = 8,
RIGHT_IN_NEST = 9,
IN_NEST = 10,
}

-- Rotation directions
ROTATION_DIRECTION = {
CLOCKWISE = 0 ,
COUNTERCLOCKWISE = 1 ,
}

-- Obstacle types
OBSTACLE_TYPE = {
NO_OBSTACLE = 0 ,
ROBOT = 1 ,
WALL = 2,
}
-- Probabilities

--Controller parameter
CONTROLLER_PARAMETER = {
WHEEL_SPEED = 10,
DISTANCE_SCANNER_THRESHOLD = 80,
SENSORS = 11,
EXPLORER_TO_BEACON_PROBABILITY = 0.05,
BEACON_NAVIGATION_THRESHOLD = 40,
SENSING_DISTANCE = 65, --[[ [cm] d_{camera} ]]
EXPLORER_DISTANCE = 15, --[[ [cm] d_{expl} ]]
CHAIN_DISTANCE = 130, --[[ [cm] d_{chain} ]]
RAB_RANGE = 150, --[[ [cm] d_{chain} ]]
DISTANCE_SCANNER_ANGULAR_SPEED = 2*math.pi,
JUNCTION_THRESHOLD = 0.1,
DEPLOYMENT_DELAY = 0, --[[ [Time steps] ]]
NO_STOP_DELAY = 100
}

RANGE = { 
RADIANS = Range:new(0,math.pi),
DISTANCE_SCANNER = Range:new(4,150),
JUNCTION_RANGE = Range:new(0,95),
}

--[[ Initialization of the state of the robot ]]
function init()
	-- Get the robot numeric id
	robotId = string.gsub(robot.id,"rescuer","")
	robotId = tonumber(robotId)
	
	reset()

end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	-- 1. Sense
	desiredDirection = Vector2:fromPolar(1,0)
	position = ParseGroundSensors()
	
	--log("Robot "..robotId..desiredDirection:toString())
	ParseRAB()
	dsRepulsionVec = ParseDistanceScanner()
	--speeds = ProximitySensorsToSpeeds()
	
	-- 2. Think
	if robot.state == STATE.CHAIN_FOLLOWING then
		ChainFollowingBehavior()
	end
	
	if robot.state == STATE.CHAIN_END then
		isStopped = true
		ChainEndBehavior()
		-- Broadcast navigation informations for the other robots
		-- Eventually LJ to preserve connection and realign chain
	end

	if robot.state == STATE.CHAIN_MEMBER then
		isStopped = true
		ChainMemberBehavior()
		-- Broadcast navigation informations for the other robots
		-- Eventually LJ to preserve connection and realign chain
	end

	if robot.state == STATE.CHAIN_JUNCTION then
		isStopped = true
		ChainJunctionBehavior()
		-- Broadcast navigation informations for the other robots
		-- Eventually LJ to preserve connection and realign chain
	end
	
	if robot.state == STATE.EXIT_NEST then
		ExitNestBehavior()
	end

	if robot.state == STATE.ATTACHING then
		AttachingBehavior()
	end

	--3. Act
	
	-- In any case, once the nest has been quitted, avoid going back to it.
	if( position == POSITION.FRONT_IN_NEST ) then
		desiredDirection = Vector2:fromPolar(2,math.pi)
		logerr("Robot "..robotId.." front in nest!")
		return
	end
	
	-- If the robot has its left part in the spot, then turn left to completely enter it.
	-- If the robot has its right part in the nest, then turn left to drive away from it.
	if (position == POSITION.LEFT_IN_SPOT) or (position == POSITION.RIGHT_IN_NEST) then
		desiredDirection = Vector2:fromPolar(2,-math.pi/2)
		logerr("Robot "..robotId.." right in nest!")
		return
	end
	
	-- If the robot has its right part in the spot, then turn right to completely enter it.
	-- If the robot has its right part in the nest, then turn left to drive away from it.
	if (position == POSITION.RIGHT_IN_SPOT) or (position == POSITION.LEFT_IN_NEST) then
		desiredDirection = Vector2:fromPolar(2,math.pi/2)
		logerr("Robot "..robotId.." left in nest!")
		return
	end

	
	-- Each robot broadcast its own ID and the encoding of its current state to all the neighboring robots
	SendMessage(robotId,robot.state.encoding,robot.chain_id,spotFound)

	-- Compute the vector sum among the desired direction and the resulting vector from the Proximity sensors reading
	desiredDirection = desiredDirection + ParseProximitySensors()
	
	if (position == POSITION.OUT_OF_NEST and robot.state == STATE.ATTACHING) or robot.state == STATE.EXIT_NEST then
		desiredDirection = desiredDirection + dsRepulsionVec
	end
	
	-- Incremental deployment
	if (stepsCounter == CONTROLLER_PARAMETER.DEPLOYMENT_DELAY * robotId) then
		isStopped = false	
	end	
	
	
	if isStopped then
		speeds = {left = 0, right = 0}
	else
		speeds = ComputeSpeedsFromAngle(desiredDirection:angle())
	end
	-- Actuate the computed velocities on the wheels
	robot.wheels.set_velocity(speeds.left, speeds.right)
	
	--log("Robot "..robotId..": "..robot.state.text)
	stepsCounter = stepsCounter + 1
end


--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
	-- Initialize controller state
	robot.state = STATE.EXIT_NEST
	robot.chain_id = 0
	sensedBeacons = {}	

	if robot.random.uniform() < 0.5 then
		rotation = ROTATION_DIRECTION.CLOCKWISE
	else
		rotation = ROTATION_DIRECTION.COUNTERCLOCKWISE
	end
	
	position = POSITION.IN_NEST
	noStopSteps = 0
	turnCounter = 0
	isStopped = true
	isLeaving = false
	spotFound = false
	leavingBeaconId = -1
	rotationCounter = -1 
	departureTime = 0
	currentChainId = -1
	targetBeaconId = -1
	readingsInALoop = 0
	readingsInJunctionRange = 0
	readingsCounter = 0
	obstaclePercentage = 1
	stepsCounter = 0

	-- Enabling of the distance scanner for further use.
	robot.distance_scanner.enable()
	-- Instead of having a rotating distance scanner, lock it in a way that the long range sensor is pointing in front of the robot
	-- In case rotation is required: robot.distance_scanner.set_rpm(10)
	--robot.distance_scanner.set_angle(math.pi/2)
	robot.distance_scanner.set_rpm(CONTROLLER_PARAMETER.DISTANCE_SCANNER_ANGULAR_SPEED)
	-- Initialize statistics
	robot.in_chain = 0
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   robot.distance_scanner.disable()	
end


--[[ Function used to compute the Lennard-Jones virtual force:
		p(d) = \epsilon * [(\frac{\sigma}{d})^12 - 2*(\frac{\sigma}{d})^6]
		 |
		F(d) = -12*\frac{\epsilon}{d} * [(\frac{\sigma}{d})^12 - (\frac{\sigma}{d})^6]
		 | (approximated by)
		F(d) = -4*\frac{\epsilon}{d} * [(\frac{\sigma}{d})^4 - (\frac{\sigma}{d})^2]
 ]]
function LJForce(distance,epsilon,sigma)
	return -4*(epsilon/distance)*(math.pow(sigma/distance,4)-math.pow(sigma/distance,2))
end

--This function computes the necessary wheel speed to go in the direction of the desired angle.
function ComputeSpeedsFromAngle(angle)
   dotProduct = 0.0;
   KProp = 20;
   wheelsDistance = 0.14;

	if angle ~= 0 then
    	-- if the target angle is behind the robot, we just rotate, no forward motion
    	if angle > math.pi/2 or angle < -math.pi/2 then
       	dotProduct = 0.0;
   	else
   	 -- else, we compute the projection of the forward motion vector with the desired angle
       	forwardVector = {math.cos(0), math.sin(0)}
       	targetVector = {math.cos(angle), math.sin(angle)}
        	dotProduct = forwardVector[1]*targetVector[1]+forwardVector[2]*targetVector[2]
    	end

	 	-- the angular velocity component is the desired angle scaled linearly
    	angularVelocity = KProp * angle;
    	-- the final wheel speeds are compute combining the forward and angular velocities, with different signs for the left and right wheel.
    	speeds = {left = dotProduct * CONTROLLER_PARAMETER.WHEEL_SPEED - angularVelocity * wheelsDistance, right = dotProduct * CONTROLLER_PARAMETER.WHEEL_SPEED + angularVelocity * wheelsDistance}
	else
	speeds = {left = CONTROLLER_PARAMETER.WHEEL_SPEED, right = CONTROLLER_PARAMETER.WHEEL_SPEED}
	end
   
	return speeds
end

--[[ Function used to send a message having the format:
		ID State 00 00 00 00 00 00 00 msg
	  using the R&B actuator
 ]]
function SendMessage(id,state,chain_id,spotFound)
	robot.range_and_bearing.set_data(1,id)
	robot.range_and_bearing.set_data(2,state)
	robot.range_and_bearing.set_data(3,chain_id)
	if spotFound then
		robot.range_and_bearing.set_data(4,1)
	else
		robot.range_and_bearing.set_data(4,0)
	end
	for i = 5 , 10 do
		robot.range_and_bearing.set_data(i,0)
	end
end

--[[ Function used to copy and sort the readings of the distance scanner according to their distance.
	  The distance scanner is a rotating device with four sensors. 
	  Two sensors are short-range (4cm to 30cm) and two are long-range (20cm to 150cm). 
     Each sensor returns up to 6 values every time step, for a total of 24 readings (12 short-range and 12 long-range). 
	  Each reading is a table composed of angle in radians and distance in cm.
	  Distance could be:
		-> -1, if the object detected by the sensor is closer than the minimum sensor range (4cm for short-range, 20cm for long-range). 
		-> -2, if no object is detected at all.  
]]
function ParseDistanceScanner()
	frontObstacleDistance = 0
	local accumulator = Vector2:fromXY(0,0)
	local dsVec = Vector2:fromPolar(0,0)
	local scalingFactor = 0
	--dsShortReadings = CopyTable(robot.distance_scanner.short_range)
   --dsLongReadings = CopyTable(robot.distance_scanner.long_range)
	--table.sort(dsShortReadings, function(a,b) return a.distance < b.distance end)
   --table.sort(dsLongReadings, function(a,b) return a.distance < b.distance end)
	for i=1,#robot.distance_scanner.long_range do 
		if robot.distance_scanner.long_range[i].distance > 0 then
			dsVec = Vector2:fromPolar(1-(RANGE.DISTANCE_SCANNER:NormalizeInRange(robot.distance_scanner.long_range[i].distance)),robot.distance_scanner.long_range[i].angle)
			dsVec:Rotate(math.pi)
			accumulator = accumulator + dsVec
			scalingFactor = scalingFactor+1
			
			if robot.distance_scanner.long_range[i].angle == 0 then
				frontObstacleDistance = robot.distance_scanner.long_range[i].distance				
			end
		
			if RANGE.JUNCTION_RANGE:WithinBoundsIncluded(robot.distance_scanner.long_range[i].angle) then
				readingsInJunctionRange = readingsInJunctionRange + 1
			end
			--[[if robot.distance_scanner.long_range[i].distance > 0 and robot.distance_scanner.long_range[i].distance < closestObstacleDS.distance then
				closestObstacleDS = robot.distance_scanner.long_range[i]
			end]]
		end
		readingsInALoop = readingsInALoop + 1
	end


	if readingsCounter > 100 then
		obstaclePercentage = readingsInJunctionRange/readingsInALoop
		readingsInALoop = 0
		readingsInJunctionRange = 0
		readingsCounter = 0
	else
		readingsCounter = readingsCounter + 1
	end

	--[[if state == STATE.CHAIN_END or 
		state == STATE.CHAIN_MEMBER or
		state == STATE.CHAIN_JUNCTION then
		if CONTROLLER_PARAMETER.DISTANCE_SCANNER_ANGULAR_SPEED
	end]]
	
	for i=1,#robot.distance_scanner.short_range do
		if robot.distance_scanner.short_range[i].distance > 0 then
			dsVec = Vector2:fromPolar(1-(RANGE.DISTANCE_SCANNER:NormalizeInRange(robot.distance_scanner.long_range[i].distance)),robot.distance_scanner.long_range[i].angle)
			dsVec:Rotate(math.pi)
			accumulator = accumulator + dsVec
			scalingFactor = scalingFactor+1 
		end			
	end

	if scalingFactor > 0 then
		accumulator:Scale(1/scalingFactor)
	end
	
	return accumulator
	
	--return Vector2:fromPolar(1,accumulator:angle())

end

--[[ Function used to parse the ground sensors: 
							2		1 
							3	 	4
	  Returns an encoding of the position of the robot with respect to the spot
	  
	]]
function ParseGroundSensors()
	local GREY_RANGE = Range:new(0.858039,0.958039)

	--[[log("Robot "..robotId..": 1 -"..robot.motor_ground[1].value)
	log("Robot "..robotId..": 2 -"..robot.motor_ground[2].value)
	log("Robot "..robotId..": 3 -"..robot.motor_ground[3].value)
	log("Robot "..robotId..": 4 -"..robot.motor_ground[4].value)]]
	
	if((robot.motor_ground[1].value == 1) and (robot.motor_ground[2].value == 1) and (robot.motor_ground[3].value == 1) and (robot.motor_ground[4].value == 1) ) then
		return POSITION.OUT_OF_NEST
	end
	
	if((GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[1].value)) and
 		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[2].value)) and
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[3].value)) and 
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[4].value))) then 
		return POSITION.IN_NEST
	end	


	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[2].value == 0) and (robot.motor_ground[3].value == 0) and (robot.motor_ground[4].value == 0) ) then
		return POSITION.IN_SPOT
	end
	
	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[2].value == 0) and
		(robot.motor_ground[3].value == 1) and (robot.motor_ground[4].value == 1)) then
		return POSITION.FRONT_IN_SPOT
	end
	
	if((robot.motor_ground[3].value == 0) and (robot.motor_ground[4].value == 0) and
		(robot.motor_ground[1].value == 1) and (robot.motor_ground[2].value == 1)) then
		return POSITION.BACK_IN_SPOT;
	end

	if((robot.motor_ground[2].value == 0) and (robot.motor_ground[3].value == 0) and
		(robot.motor_ground[1].value == 1) and (robot.motor_ground[4].value == 1)) then
		return POSITION.RIGHT_IN_SPOT 
	end

	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[4].value == 0) and
		(robot.motor_ground[2].value == 1) and (robot.motor_ground[3].value == 1)) then
		return POSITION.LEFT_IN_SPOT

	end

	if((GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[1].value)) and
 		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[2].value)) and
		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[3].value)) and 
		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[4].value))) then 
		return POSITION.FRONT_IN_NEST;
	end
	
	if(not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[1].value)) and
 		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[2].value)) and
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[3].value)) and 
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[4].value))) then 
		return POSITION.BACK_IN_NEST;
	end

	if(not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[1].value)) and
 		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[2].value)) and
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[3].value)) and 
		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[4].value))) then 
		return POSITION.RIGHT_IN_NEST
	end

	if((GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[1].value)) and
 		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[2].value)) and
		not (GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[3].value)) and 
		(GREY_RANGE:WithinBoundsIncluded(robot.motor_ground[4].value))) then
		return POSITION.LEFT_IN_NEST
	end

	return POSITION.OUT_OF_NEST;

	
end

--[[ Function used to copy and sort the readings of the proximity sensors according to their distance.
     The sensors are 24 and are equally distributed in a ring around the robot body. 
	  In Lua they are indexed from 1 to 25, with the index value growing counterclockwise.
	  Each sensor has a range of 10cm and returns a reading composed of an angle in radians and a value in the range [0,1]. 
	  The angle corresponds to where the sensor is located in the body with respect to the front of the robot, which is the local x axis. 
     The value is 0 if no obstacle is sensed and increases as the robot gets closer to the object. 
]]
function ParseProximitySensors()
	local accumulator = Vector2:fromXY(0,0)
	if not (robot.proximity == nil) then
		--proximityReadings = CopyTable(robot.proximity)
		for i=1,#robot.proximity do 
			--Sum up individual forces in accumulator vector
			local proximityVec = Vector2:fromPolar(robot.proximity[i].value,robot.proximity[i].angle)
			--proximityVec:Scale(1/#proximityReadings)
			proximityVec:Rotate(math.pi)
			accumulator = accumulator + proximityVec
		end
		-- Normalization of the accumulator
		--accumulator = Vector2:fromPolar(accumulator:length(),accumulator:angle())
		accumulator = Vector2:fromPolar(1.5,accumulator:angle())
		--log(robotId.." "..accumulator:toString())
	end
	return accumulator
end

function ProximitySensorsToSpeeds()
	local speeds = {left=CONTROLLER_PARAMETER.WHEEL_SPEED,right=CONTROLLER_PARAMETER.WHEEL_SPEED}
	closestObstaclePS = {value=-1, index=-1}
	if not (robot.proximity == nil) then
		for i=1,#robot.proximity do 
			--Find maximum reading
			if robot.proximity[i].value > closestObstaclePS.value then
				closestObstaclePS.value = robot.proximity[i].value
				closestObstaclePS.index = i
			end
		end
		if closestObstaclePS.value ~= 0 then
			if closestObstaclePS.index <= 12 then
				-- The closest obstacle is between 0 and 180 degrees: soft turn towards the right
				speeds = {left=CONTROLLER_PARAMETER.WHEEL_SPEED, right=((closestObstaclePS.index-1)*CONTROLLER_PARAMETER.WHEEL_SPEED/CONTROLLER_PARAMETER.SENSORS)}
			else
				-- The closest obstacle is between 180 and 360 degrees: soft turn towards the left
				speeds = {left=((24-closestObstaclePS.index)*CONTROLLER_PARAMETER.WHEEL_SPEED/CONTROLLER_PARAMETER.SENSORS), right=CONTROLLER_PARAMETER.WHEEL_SPEED}
			end
		end
	
	end
	return speeds
end


--[[ Function used to copy and sort the readings of the range and bearing sensors according to their distance.
     Each message is stored in a table composed of:
		# data (the 10-bytes message payload)
		# horizontal_bearing (the angle between the robot local x axis and the position of the message source; the angle is on the robot's xy plane, in radians), 				 
		# vertical_bearing (like the horizontal bearing, but it is the angle between the message source and the robot's xy plane)
		# range (the distance of the message source in cm).
]]
function ParseRAB()
	closestBeacon = {id = -1, chain_id = 0,  distance = 151, angle = 0 , type = -1}
	farthestBeacon = {id = -1, chain_id = 0,  distance = 0, angle = 0 , type = -1}
	robot.neighbors = {in_nest=0, attaching = 0, following = 0, chain_ends = 0, chain_members = 0,  chain_junctions = 0, chain_beacons = 0}
	sensedBeacons = {}

	for i=1,#robot.range_and_bearing do 
		if (robot.range_and_bearing[i]).data[2] == STATE.EXIT_NEST.encoding then
			robot.neighbors.in_nest = robot.neighbors.in_nest + 1 
		end
		
		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_FOLLOWING.encoding then
			robot.neighbors.following = robot.neighbors.following + 1 
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.ATTACHING.encoding then
			robot.neighbors.attaching = robot.neighbors.attaching + 1 
		end
		
		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_END.encoding then
			robot.neighbors.chain_ends = robot.neighbors.chain_ends + 1
			sensedBeacons [(robot.range_and_bearing[i].data[1])] = 1
			sensedBeacons[(robot.range_and_bearing[i].data[1])] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2],
																						spotFound = (robot.range_and_bearing[i]).data[4] }
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_MEMBER.encoding then
			robot.neighbors.chain_members = robot.neighbors.chain_members + 1
			sensedBeacons[(robot.range_and_bearing[i].data[1])] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2],
																						spotFound = (robot.range_and_bearing[i]).data[4] }
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_JUNCTION.encoding then
			robot.neighbors.chain_junctions = robot.neighbors.chain_junctions + 1
			sensedBeacons[(robot.range_and_bearing[i].data[1])] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2],
																						spotFound = (robot.range_and_bearing[i]).data[4] }
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_JUNCTION.encoding or 
			(robot.range_and_bearing[i]).data[2] == STATE.CHAIN_MEMBER.encoding or 
			(robot.range_and_bearing[i]).data[2] == STATE.CHAIN_END.encoding  then

			if (robot.range_and_bearing[i]).range < closestBeacon.distance then
					closestBeacon = {id = (robot.range_and_bearing[i]).data[1] , 
										  chain_id = (robot.range_and_bearing[i]).data[3], 
										  distance = (robot.range_and_bearing[i]).range, 
										  angle = 	(robot.range_and_bearing[i]).horizontal_bearing, 
										  type = (robot.range_and_bearing[i]).data[2],
										  spotFound = (robot.range_and_bearing[i]).data[4] }
			end
			
			if (robot.range_and_bearing[i]).range > farthestBeacon.distance then
					farthestBeacon = {id = (robot.range_and_bearing[i]).data[1] , 
										  chain_id = (robot.range_and_bearing[i]).data[3], 
										  distance = (robot.range_and_bearing[i]).range, 
										  angle = 	(robot.range_and_bearing[i]).horizontal_bearing, 
										  type = (robot.range_and_bearing[i]).data[2],
										  spotFound = (robot.range_and_bearing[i]).data[4] }
			end
		end
	
	end
	robot.neighbors.chain_beacons = robot.neighbors.chain_members +
												robot.neighbors.chain_junctions +
												robot.neighbors.chain_ends 
end


function AttachingBehavior()

	--[[if robot.neighbors.chain_beacons > 0 then
		robot.state = STATE.DIRECTING_TO_BEACON
		log("Robot "..robotId.." direction beacon!")
		return 
	end]]

	
				desiredDirection:SetFromPolar(1.2,closestBeacon.angle)
				desiredDirection:Rotate(math.pi)
				return 
		
	
	if position ==	POSITION.IN_NEST then
		robot.state = STATE.EXIT_NEST
		return
	end

	--[[if position == POSITION.OUT_OF_NEST then
		robot.distance_scanner.set_rpm(CONTROLLER_PARAMETER.DISTANCE_SCANNER_ANGULAR_SPEED)
	end]]
	if position == POSITION.OUT_OF_NEST and 
		(robot.neighbors.chain_beacons == 1 or closestBeacon.distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE) then
			robot.state = STATE.CHAIN_END
			-- Compute the robot chain ID by reading the neighboring robot ID and incrementing it by one.
			if robot.neighbors.chain_beacons > 0 then
				robot.chain_id = sensedBeacons[1].chain_id + 1
			else
				robot.chain_id = -1
			end
			leavingBeaconId = -1
			return
	end]]

	--if robot.neighbors.explorers > 0 then
	
	--end


	if noStopSteps < 0 then	
		if robot.neighbors.chain_beacons == 0 and not isLeaving then	
			if robot.random.uniform() < CONTROLLER_PARAMETER.EXPLORER_TO_BEACON_PROBABILITY and position == POSITION.OUT_OF_NEST then
				robot.state = STATE.CHAIN_END
				log("Robot "..robotId..": Chain end.")
				isLeaving = false
				robot.distance_scanner.set_rpm(0)
				robot.in_chain = 1
				return
			end
		end
	else
		noStopSteps = noStopSteps - 1
	end

end

function ChainEndBehavior()

	--[[if obstaclePercentage < CONTROLLER_PARAMETER.JUNCTION_THRESHOLD and robot.state ~= STATE.CHAIN_JUNCTION then
		robot.state = STATE.CHAIN_JUNCTION
		log("Robot "..robotId..": Chain junction.")
		return
	end]]
	
	--[[if robot.neighbors.chain_beacons == 1 then
		isStopped = false
		desiredDirection:SetFromPolar(1,closestBeacon.angle)
		desiredDirection:Rotate(math.pi/4)
	end]]

	if robot.neighbors.chain_beacons > 1 then
		robot.state = STATE.CHAIN_MEMBER
		log("Robot "..robotId..": Chain member.")
		return
	end	
end

function ChainMemberBehavior()
	--If the following beacon in the chain, has found the spot, then backpropagate the information
	for key,value in pairs(sensedBeacons) do
		if value.spotFound and value.chain_id > robot.chain_id then
			if not spotFound then
				spotFound = true
			end
		end
	end

	--[[if obstaclePercentage < CONTROLLER_PARAMETER.JUNCTION_THRESHOLD and robot.state ~= STATE.CHAIN_JUNCTION then
		robot.state = STATE.CHAIN_JUNCTION
		log("Robot "..robotId..": Chain junction.")
		return
	end]]

end

function ChainJunctionBehavior()
	--The backpropagation should stop once a junction beacon is reached
	--[[for key,value in pairs(sensedBeacons) do
		if value.spotFound and value.chain_id > robot.chain_id then
			if not spotFound then
				spotFound = true
			end
		end
	end]]
end

function ExitNestBehavior()
	
	dsRepulsionVec = Vector2:fromPolar(1.0,dsRepulsionVec:angle())

	-- If the robot senses any type of beacons, it direct itself towards it
	if robot.neighbors.chain_beacons > 0 then
		log("Robot "..robotId.." direction beacon!")
		robot.state = STATE.CHAIN_FOLLOWING
		targetBeaconId = closestBeacon.id  
		return 
	end
	
	
	-- If the robot has left the nest, then it becomes an explorer.
	if( position == POSITION.OUT_OF_NEST ) then
		robot.state = STATE.ATTACHING
		noStopSteps = CONTROLLER_PARAMETER.NO_STOP_DELAY
		log("Robot "..robotId.." attaching!")
		return
	end]]

	--If the robot is going straight..
	--if desiredDirection:angle() then
	-- ...and it is trying to exit nest, take into account the readings from the distance scanner
		--[[if frontObstacleDistance > 0 and frontObstacleDistance < CONTROLLER_PARAMETER.DISTANCE_SCANNER_THRESHOLD then
			turnCounter = 0
			local rotationAngle = math.pi
			--local rotationAngle = RADIANS_RANGE.UniformRandom()
			--log("Robot "..robotId.." long distance wall!")
			if rotation == ROTATION_DIRECTION.CLOCKWISE then
				desiredDirection:SetFromPolar(1,-rotationAngle)
			end
			
			if rotation == ROTATION_DIRECTION.COUNTERCLOCKWISE then
				desiredDirection:SetFromPolar(1,rotationAngle)
			end
		else
			turnCounter = turnCounter + 1
			
			if(turnCounter > 90) then
				if rotation == ROTATION_DIRECTION.CLOCKWISE then
					desiredDirection:SetFromPolar(1,math.pi/2)
				end
				if rotation == ROTATION_DIRECTION.COUNTERCLOCKWISE then
					desiredDirection:SetFromPolar(1,-math.pi/2)
				end
				if(turnCounter > robot.random.uniform(110,125)) then 
					turnCounter = 0
				end
			end
		end
		return]]
	--end
	--if closestObstacleDS.distance > 0  then
		--desiredDirection = ParseProximitySensors() + Vector2:fromPolar(1,closestObstacleDS.angle)
		-- To check whether this will suffice or rotation + moving straight is required
	--end
	
end

function ChainFollowingBehavior()
		local maxId = -1
		local maxSensedChainId = -1
		local targetExists = false

		-- Stop if the robot has reached the spot
		if( position == POSITION.IN_SPOT ) then
			robot.state = STATE.SPOT_REACHED
			isStopped = true
			spotFound = true
		end

		if robot.neighbors.chain_beacons == 0 then
			if position == POSITION.IN_NEST then
				robot.state = STATE.EXIT_NEST
				return
			end
			if position == POSITION.OUT_OF_NEST then
				isStopped = true
				return
			end
		end
		
		for key,value in pairs(sensedBeacons) do
				if value.chain_id >= maxSensedChainId then 
					maxId = key
					maxSensedChainId = value.chain_id
				end

				if key == targetBeaconId then
					targetBeaconID = key
					targetExists = true
				end
				
				if value.type == STATE.CHAIN_END.encoding then
					if value.distance < CONTROLLER_PARAMETER.SENSING_DISTANCE then 
						targetBeaconID = key
						robot.state = STATE.ATTACHING 
						return
					end
				end
		end

		if maxSensedChainId ~= -1 then
			if maxSensedChainId > currentChainId then
				targetBeaconID = maxId
				currentChainId = maxSensedChainId
			end
			desiredDirection:SetFromPolar(1.0,sensedBeacons[targetBeaconID].angle)
			return
		end

		if targetExists then
			desiredDirection:SetFromPolar(1.0,sensedBeacons[targetBeaconId].angle)
			return
		end

		

end
		
