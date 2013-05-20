require "SIUtils"

-- Sensors: 
-- # Distance scanner
-- # Ground sensors
-- # Proximity sensors

-- Actuators:
-- # Wheels
-- # Range and bearing system

-- Statistics

robot.in_chain = 0

-- States: 
WALK = "Linear walk"
LTURN = "Left turn"
RTURN = "Right turn"
SPOT = "Entered spot"
STOP = "Stop"

-- Readings
IN_SPOT = 0;
FRONT_IN_SPOT = 1;
BACK_IN_SPOT = 2;
LEFT_IN_SPOT = 3;
RIGHT_IN_SPOT = 4;
NOT_IN_SPOT = 5;

-- Probabilities

-- P_{stop} = P_{ss} + alpha*NeighborCount + P_{bs} 
-- Probability of stopping spontaneously
P_ss = 0.05
-- Alpha
alpha = 0.05
-- Probability of stopping while being in the black spot
P_bs = 0


-- P_{walk} = P_{sw} + beta*NeighborCount + P_{bw}
-- Probability of walking spontaneously
P_sw = 0.05
--Beta
beta = 0.05
-- Probability of stopping while being in the black spot
P_bw = 0


--Thresholds
rbThreshold = 0.5



--[[ Initialization of the state of the robot ]]
function init()
   state = WALK
	randomTurnSteps = 0
	basicVelocity = 3
	randomInertia = 0
	randomTurnSteps = 0
	leftObstacle = false
	rightObstacle = false
	neighborCount = 0
	robotId = string.gsub(robot.id,"fb","")
	robotId = tonumber(robotId)
	-- Enabling of the distance scanner for further use.
	robot.distance_scanner.enable()
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
   -- put your code here
end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
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
function ComputeSpeedFromAngle(angle)
    dotProduct = 0.0;
    KProp = 20;
    wheelsDistance = 0.14;

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
    speeds = {dotProduct * WHEEL_SPEED - angularVelocity * wheelsDistance, dotProduct * WHEEL_SPEED + angularVelocity * wheelsDistance}

    return speeds
end

--[[ Function used to send a message having the format:
		00 00 00 00 00 00 00 00 00 msg
	  using the R&B actuator
 ]]
function SendMessage(msg)
	for i = 1 , 9 do
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
	dsReadings = CopyTable(robot.distance_scanner)
	table.sort(dsReadings, function(a,b) return a.distance < b.distance)
end

--[[ Function used to parse the ground sensors: 
							2		1 
							3	 	4
	  Returns an encoding of the position of the robot with respect to the spot
	]]
function ParseGroundSensors()
	
	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[2].value == 0) and (robot.motor_ground[3].value == 0) and (robot.motor_ground[4].value == 0) ) then
		return IN_SPOT;
	end
	
	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[2].value == 0)) then
		return FRONT_IN_SPOT;
	end
	
	if((robot.motor_ground[3].value == 0) and (robot.motor_ground[4].value == 0)) then
		return BACK_IN_SPOT;
	end

	if((robot.motor_ground[2].value == 0) and (robot.motor_ground[3].value == 0)) then
		return LEFT_IN_SPOT;
	end

	if((robot.motor_ground[1].value == 0) and (robot.motor_ground[4].value == 0)) then
		return RIGHT_IN_SPOT;
	end

	return NOT_IN_SPOT;
end

--[[ Function used to copy and sort the readings of the proximity sensors according to their distance.
     The sensors are 24 and are equally distributed in a ring around the robot body. 
	  In Lua they are indexed from 1 to 25, with the index value growing counterclockwise.
	  Each sensor has a range of 10cm and returns a reading composed of an angle in radians and a value in the range [0,1]. 
	  The angle corresponds to where the sensor is located in the body with respect to the front of the robot, which is the local x axis. 
     The value is 0 if no obstacle is sensed and increases as the robot gets closer to the object. 
]]
function ParseProximitySensors()
	proximityReadings = CopyTable(robot.proximity)
	table.sort(proximityReadings, function(a,b) return a.value < b.value)
end

--[[ Function used to copy and sort the readings of the range and bearing sensors according to their distance.
     Each message is stored in a table composed of:
		# data (the 10-bytes message payload)
		# horizontal_bearing (the angle between the robot local x axis and the position of the message source; the angle is on the robot's xy plane, in radians), 				 		
		# vertical_bearing (like the horizontal bearing, but it is the angle between the message source and the robot's xy plane)
		# range (the distance of the message source in cm).
]]
function ParseRAB()
	rabReadings = CopyTable(robot.range_and_bearing)
	table.sort(rabReadings, function(a,b) return a.range < b.range)
   return #(rabReadings)
end

