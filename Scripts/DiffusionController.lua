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
EXPLORER = {text="Explorer", encoding=5},
DIRECTING_TO_BEACON = {text="Directing to Beacon", encoding=6}
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

-- P_{stop} = P_{ss} + alpha*NeighborCount + P_{bs} 
-- Probability of stopping spontaneously
--P_ss = 0.05
-- Alpha
--alpha = 0.05
-- Probability of stopping while being in the black spot
--P_bs = 0


-- P_{walk} = P_{sw} + beta*NeighborCount + P_{bw}
-- Probability of walking spontaneously
--P_sw = 0.05
--Beta
--beta = 0.05
-- Probability of stopping while being in the black spot
--P_bw = 0


--Controller parameter
CONTROLLER_PARAMETER = {
RAB_THRESHOLD = 0.5 ,
WHEEL_SPEED = 10,
RANDOM_TURN_PROBABILITY = 0.1 ,
DISTANCE_SCANNER_THRESHOLD = 80,
SENSORS = 11,
EXPLORER_TO_BEACON_PROBABILITY = 0.05,
DELTA_RANGE = (math.pi/24)/2,
BEACON_NAVIGATION_THRESHOLD = 40,
SENSING_DISTANCE = 65, --[[ [cm] d_{camera} ]]
EXPLORER_DISTANCE = 15, --[[ [cm] d_{expl} ]]
CHAIN_DISTANCE = 130 --[[ [cm] d_{chain} ]]
}

RADIANS_RANGE = Range:new(0,math.pi)

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
	ParseDistanceScanner()
	
	--speeds = ProximitySensorsToSpeeds()
	
	-- 2. Think
	if robot.state == STATE.EXPLORER then
		ExplorerBehavior()
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

	if robot.state == STATE.DIRECTING_TO_BEACON then
		DirectingToBeaconBehavior()
	end

	--3. Act
	-- Each robot broadcast its own ID and the encoding of its current state to all the neighboring robots
	SendMessage(robotId,robot.state.encoding,robot.chain_id,0)
	
	-- In any case, once the nest has been quitted, avoid going back to it.
	if( position == POSITION.FRONT_IN_NEST ) then
		desiredDirection:Rotate(math.pi)
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
	
	-- Compute the vector sum among the desired direction and the resulting vector from the Proximity sensors reading
	desiredDirection = desiredDirection + ParseProximitySensors()
	if isStopped then
		speeds = {left = 0, right = 0}
	else
		speeds = ComputeSpeedsFromAngle(desiredDirection:angle())
	end
	-- Actuate the computed velocities on the wheels
	robot.wheels.set_velocity(speeds.left, speeds.right)
	
	--log("Robot "..robotId..": "..robot.state.text)

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
	if robot.random.uniform() < 0.5 then
		rotation = ROTATION_DIRECTION.CLOCKWISE
	else
		rotation = ROTATION_DIRECTION.COUNTERCLOCKWISE
	end
	
	position = POSITION.IN_NEST
	noStopSteps = 0
	turnCounter = 0
	isStopped = false
	isLeaving = false
	leavingBeaconId = -1
	rotationCounter = -1 
	departureTime = 0
	currentChainId = 0

	-- Enabling of the distance scanner for further use.
	robot.distance_scanner.enable()
	-- Instead of having a rotating distance scanner, lock it in a way that the long range sensor is pointing in front of the robot
	-- In case rotation is required: robot.distance_scanner.set_rpm(10)
	robot.distance_scanner.set_angle(math.pi/2)
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
function SendMessage(id,state,chain_id,msg)
	robot.range_and_bearing.set_data(1,id)
	robot.range_and_bearing.set_data(2,state)
	robot.range_and_bearing.set_data(3,chain_id)
	for i = 4 , 9 do
		robot.range_and_bearing.set_data(i,0)
	end
	robot.range_and_bearing.set_data(10,msg)
end

--[[ Function used to copy and sort the readings of the distance scanner according to their distance.
	  The distance scanner is a rotating device with four sensors. 
	  Two sensors are short-range (4cm to 30cm) and two are long-range (20cm to 150cm). 
     Each sensor returns up to 6 values every time step, for a total of 24 readings (12 short-range and 12 long-range). 
	  Each reading is a table composed of angle in radians and distance in cm.  ]]
function ParseDistanceScanner()
	frontObstacleDistance = 0
	closestWall = {angle=math.pi/2, distance = 30}
	closestObstacleDS = {angle = 0, distance = 150}
	--dsShortReadings = CopyTable(robot.distance_scanner.short_range)
   --dsLongReadings = CopyTable(robot.distance_scanner.long_range)
	--table.sort(dsShortReadings, function(a,b) return a.distance < b.distance end)
   --table.sort(dsLongReadings, function(a,b) return a.distance < b.distance end)
	for i=1,#robot.distance_scanner.long_range do 
			if robot.distance_scanner.long_range[i].angle == 0 then
				frontObstacleDistance = robot.distance_scanner.long_range[i].distance
			end
			if robot.distance_scanner.long_range[i].distance > 0 and robot.distance_scanner.long_range[i].distance < closestObstacleDS.distance then
				closestObstacleDS = robot.distance_scanner.long_range[i]
			end
	end
	
	for i=1,#robot.distance_scanner.short_range do 
			if robot.distance_scanner.short_range[i].distance > 0 and robot.distance_scanner.short_range[i].distance < closestWall.distance then
				closestWall = robot.distance_scanner.short_range[i]
			end
	end

	--if #dsLongReadings ~= 0 then
		--[[angle = dsLongReadings[1].angle]]
	--end
	--return Vector2:fromPolar(1,angle)
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
			proximityVec = Vector2:fromPolar(robot.proximity[i].value,robot.proximity[i].angle)
			--proximityVec:Scale(1/#proximityReadings)
			proximityVec:Rotate(math.pi)
			accumulator = accumulator + proximityVec
		end
		-- Normalization of the accumulator
		--accumulator = Vector2:fromPolar(accumulator:length(),accumulator:angle())
		accumulator = Vector2:fromPolar(1,accumulator:angle())
		--log(robotId.." "..accumulator:toString())
		--table.sort(proximityReadings, function(a,b) return a.value < b.value end)
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
	robot.neighbors = {in_nest=0, explorers = 0, to_beacon = 0, chain_ends = 0, chain_members = 0,  chain_junctions = 0}
	sensedBeacons = {}


	for i=1,#robot.range_and_bearing do 
		if (robot.range_and_bearing[i]).data[2] == STATE.EXIT_NEST.encoding then
			robot.neighbors.in_nest = robot.neighbors.in_nest + 1 
		end
		
		if (robot.range_and_bearing[i]).data[2] == STATE.EXPLORER.encoding then
			robot.neighbors.explorers = robot.neighbors.explorers + 1 
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.DIRECTING_TO_BEACON.encoding then
			robot.neighbors.to_beacon = robot.neighbors.to_beacon + 1 
		end
		
		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_END.encoding then
			robot.neighbors.chain_ends = robot.neighbors.chain_ends + 1
			
			sensedBeacons[(robot.range_and_bearing[i]).data[1]] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2]}
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_MEMBER.encoding then
			robot.neighbors.chain_members = robot.neighbors.chain_members + 1
			
			sensedBeacons[(robot.range_and_bearing[i]).data[1]] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2]}
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_JUNCTION.encoding then
			robot.neighbors.chain_junctions = robot.neighbors.chain_junctions + 1
			sensedBeacons[(robot.range_and_bearing[i]).data[1]] = {chain_id = (robot.range_and_bearing[i]).data[3], 	
																					   distance = (robot.range_and_bearing[i]).range, 
																						angle = (robot.range_and_bearing[i]).horizontal_bearing, 
																						type = (robot.range_and_bearing[i]).data[2]}
		end

		if (robot.range_and_bearing[i]).data[2] == STATE.CHAIN_JUNCTION.encoding or 
			(robot.range_and_bearing[i]).data[2] == STATE.CHAIN_MEMBER.encoding or 
			(robot.range_and_bearing[i]).data[2] == STATE.CHAIN_END.encoding  then

			if (robot.range_and_bearing[i]).range < closestBeacon.distance then
					closestBeacon = {id = (robot.range_and_bearing[i]).data[1] , 
										  chain_id = (robot.range_and_bearing[i]).data[3], 
										  distance = (robot.range_and_bearing[i]).range, 
										  angle = 	(robot.range_and_bearing[i]).horizontal_bearing, 
										  type = (robot.range_and_bearing[i]).data[2]}
			end
			
			if (robot.range_and_bearing[i]).range > farthestBeacon.distance then
					farthestBeacon = {id = (robot.range_and_bearing[i]).data[1] , 
										  chain_id = (robot.range_and_bearing[i]).data[3], 
										  distance = (robot.range_and_bearing[i]).range, 
										  angle = 	(robot.range_and_bearing[i]).horizontal_bearing, 
										  type = (robot.range_and_bearing[i]).data[2]}
			end
		end
	
	end
	 
end


function ExplorerBehavior()

	--[[if robot.neighbors.chain_beacons > 0 then
		robot.state = STATE.DIRECTING_TO_BEACON
		return 
	end

	if robot.neighbors.chain_beacons == 1 then	
			if closestBeacon.distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE and position ~= POSITION.IN_NEST then
				robot.state = STATE.CHAIN_BEACON
				isLeaving = false
				robot.in_chain = 1
				return
			end
		end
	
	if position ==	POSITION.IN_NEST then
		robot.state = STATE.EXIT_NEST
		return
	end]]
	
	if position ==	POSITION.IN_NEST then
		robot.state = STATE.EXIT_NEST
		return
	end

	if position == POSITION.OUT_OF_NEST and 
		(#sensedBeacons == 1 or closestBeacon.distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE) then
			robot.state = STATE.CHAIN_END
			if #sensedBeacons > 0 then
				robot.chain_id = sensedBeacons[1].chain_id + 1
			else
				robot.chain_id = 0
			end
			leavingBeaconId = -1
			return
	end
	
	if #sensedBeacons > 0 then
		robot.state = STATE.DIRECTING_TO_BEACON
		return 
	end

	--if robot.neighbors.explorers > 0 then
	
	--end

	-- Stop if the robot has reached the spot
	if( position == POSITION.IN_SPOT ) then
		robot.state = STATE.SPOT_REACHED
		isStopped = true
	end


	if noStopSteps < 0 then	
		if robot.neighbors.chain_beacons == 0 and not isLeaving then	
			if robot.random.uniform() < CONTROLLER_PARAMETER.EXPLORER_TO_BEACON_PROBABILITY and position == POSITION.OUT_OF_NEST then
				robot.state = STATE.CHAIN_END
				log("Robot "..robotId..": Chain end.")
				isLeaving = false
				robot.in_chain = 1
				return
			end
		end
	else
		noStopSteps = noStopSteps - 1
	end

end

function ChainEndBehavior()
	if robot.neighbors.chain_ends + robot.neighbors.chain_members > 1 then
		robot.state = STATE.CHAIN_MEMBER
		log("Robot "..robotId..": Chain member.")
		return
	end	
end

function ChainMemberBehavior()
	if robot.neighbors.chain_ends + robot.neighbors.chain_members > 2 then
		robot.state = STATE.CHAIN_JUNCTION
		log("Robot "..robotId..": Chain junction.")
		return
	end
end

function ChainJunctionBehavior()

end

function ExitNestBehavior()
	
	-- If the robot senses any type of beacons, it direct itself towards it
	if #sensedBeacons > 0 then
		robot.state = STATE.DIRECTING_TO_BEACON
		targetBeaconId = closestBeacon.id  
		return 
	end
	
	
	-- If the robot has left the nest, then it becomes an explorer.
	if( position == POSITION.OUT_OF_NEST ) then
		robot.state = STATE.EXPLORER
		noStopSteps = 100
		log("Robot "..robotId.." exploring!")
		-- Since no camera is available, the research of a beacon will be made using the distance scanner
		--robot.distance_scanner.set_rpm(1)
		return
	end

	--If the robot is going straight..
	if desiredDirection:angle() then
	-- ...and it is trying to exit nest, take into account the readings from the distance scanner
		if frontObstacleDistance > 0 and frontObstacleDistance < CONTROLLER_PARAMETER.DISTANCE_SCANNER_THRESHOLD then
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
		return
	end
	--if closestObstacleDS.distance > 0  then
		--desiredDirection = ParseProximitySensors() + Vector2:fromPolar(1,closestObstacleDS.angle)
		-- To check whether this will suffice or rotation + moving straight is required
	--end
	
end

function DirectingToBeaconBehavior()
		local maxSensedChainId = 0

		if #sensedBeacons == 0 then
			if position == POSITION.IN_NEST then
				robot.state = STATE.EXIT_NEST
				return
			end
			if position == POSITION.OUT_OF_NEST then
				robot.state = STATE.EXPLORER
				return
			end
		end

		if #sensedBeacons == 1 then
			if isLeaving then
				if position == POSITION.OUT_OF_NEST and sensedBeacons[1].distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE then
					robot.state = STATE.CHAIN_END
					robot.chain_id = sensedBeacons[1].chain_id + 1
					log("Robot "..robotId..": Chain end.")
					leavingBeaconId = -1
					return
				else
					desiredDirection:SetFromPolar(0.9,sensedBeacons[1].angle)
					desiredDirection:Rotate(math.pi)
					return
				end
			end

			if sensedBeacons[1] == STATE.CHAIN_END.encoding then
				if sensedBeacons[1].distance > CONTROLLER_PARAMETER.SENSING_DISTANCE then
					rotationCounter = 0
					desiredDirection:SetFromPolar(0.9,sensedBeacons[1].angle)
					currentChainId = sensedBeacons[1].chain_id
					return
				end
			else
				if rotationCounter == 0 then
					departureTime = robot.random.uniform(30,120)
				end
				
				if rotationCounter > departureTime then
					isLeaving = true
					leavingBeaconId = closestBeacon.id
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi)
				else
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					if rotation == ROTATION_DIRECTION.CLOCKWISE then
						desiredDirection:Rotate(-math.pi/4)
					end
					if rotation == ROTATION_DIRECTION.COUNTERCLOCKWISE then
						desiredDirection:Rotate(math.pi/4)
					end
					rotationCounter = rotationCounter + 1
				end
				return	
			end
			
			if sensedBeacons[1] == STATE.CHAIN_MEMBER.encoding then
			end
			if sensedBeacons[1] == STATE.CHAIN_BEACON.encoding then
			end
		end

		if #sensedBeacons > 1 then
			for key,value in pairs(sensedBeacons) do
				if value.chain_id >= maxSensedChainId then 
					targetBeaconId = key
					maxSensedChainId = value.chain_id 
					break
				end
			end
			if maxSensedChainId >= currentChainId then
				targetBeacon = sensedBeacons[targetBeaconId]
			else
				targetBeacon = farthestBeacon
			end
			desiredDirection:SetFromPolar(0.9,targetBeacon.angle) 
			return
		end
end
		--[[targetBeacon = sensedBeacons[targetBeaconId]
		if targetBeacon == nil then
			if position == POSITION.IN_NEST then
				robot.state = STATE.EXIT_NEST
				return
			end
		end
		currentChainId = targetBeacon.chain_id
		
		if position == POSITION.IN_NEST and ( leavingBeaconId > 0 or targetBeacon.id == -1)  then
			robot.state = STATE.EXIT_NEST
			leavingBeaconId = -1
			return
		end

		if position == POSITION.OUT_OF_NEST and (robot.neighbors.chain_beacons == 1 and targetBeacon.distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE) then
				robot.state = STATE.CHAIN_END
				robot.chain_id = currentChainId + 1
				log("Robot "..robotId..": Chain end.")
				leavingBeaconId = -1
				return
		end
		
		if isLeaving then
			if position == POSITION.OUT_OF_NEST and (robot.neighbors.chain_beacons == 1 and targetBeacon.distance > CONTROLLER_PARAMETER.CHAIN_DISTANCE) then
				robot.state = STATE.CHAIN_END
				robot.chain_id = currentChainId + 1
				log("Robot "..robotId..": Chain end.")
				leavingBeaconId = -1
				return
			else
				if #sensedBeacons > 1 then
					for key,value in pairs(sensedBeacons) do
						if value.chain_id > currentChainId then 
							targetBeaconId = key
							currentChainId = value.chain_id 
							break
						end
					end
					targetBeacon = sensedBeacons[targetBeaconId]
					--log("Robot "..robotId..": Leaving beacon with more than one beacon sensed.")
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				else
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi)
				end
				return
			end
		end
	
		--If the closest beacon is the end of the chain, rotate around it, then go away from it after a random rotation time.	
		if targetBeacon.type == STATE.CHAIN_END.encoding then
			log("Robot "..robotId..": Directing to chain end.")
			if targetBeacon.distance > CONTROLLER_PARAMETER.SENSING_DISTANCE then
				rotationCounter = 0
				desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				return
			else
				if rotationCounter == 0 then
					departureTime = robot.random.uniform(30,120)
				end
				if rotationCounter > departureTime then
					isLeaving = true
					leavingBeaconId = closestBeacon.id
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi)
				else
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi/6)
					rotationCounter = rotationCounter + 1
				end
				return	
			end
	 end

	-- If the closest beacon is a chain member, then follow the chain until its end
	if targetBeacon.type == STATE.CHAIN_MEMBER.encoding then
			if targetBeacon.distance > CONTROLLER_PARAMETER.SENSING_DISTANCE then
				desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				return
			else
				if #sensedBeacons > 1 then
					log("Robot "..robotId..": Following chain member with more than one beacon sensed.")
					isLeaving = true
					--targetBeaconID = sensedBeacons[1].id
					--desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				else
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi/4)
				end
				
				return	
			end
	 end

	-- If the closest beacon is a junction, rotate randomly around it and then choose wh
	if targetBeacon.type == STATE.CHAIN_JUNCTION.encoding then
			if targetBeacon.distance > CONTROLLER_PARAMETER.SENSING_DISTANCE then
				desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				return
			else
				if #sensedBeacons > 1 then
					log("Robot "..robotId..": Following junction with more than one beacon sensed.")
					isLeaving = true
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
				else
					desiredDirection:SetFromPolar(0.9,targetBeacon.angle)
					desiredDirection:Rotate(math.pi/4)
				end
				return	
			end
	 end]]
