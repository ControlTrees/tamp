Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
free		# a location is free
in_sight	# a location is visible
behind		# a vehicle is behind another one
before		# a vehicle X is in front of antoher one Y
observed        # a scene has been observed

# keyword
NOT_OBSERVABLE

## constants
observed
following
vehicle
location
scene
ego_car
truck
lane_2
lanes

## initial state
START_STATE { 
(vehicle ego_car) (vehicle truck) 
(behind  ego_car truck)
(scene   lanes)
(following)
}



