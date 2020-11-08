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
lane_1_near
lane_1_far
lane_2_near
lane_2_far
lanes

## initial state
START_STATE { 
(vehicle ego_car) (vehicle truck) 
(behind  ego_car truck)
(scene   lanes)
(following)
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE free lane_1_near)
(NOT_OBSERVABLE free lane_1_far)
}
}

BELIEF_START_STATE{ 
{
()=1.0
}
}

### Termination RULES 
Rule {
  { (before ego_car truck) } # 
  { (QUIT) }
}

Rule {
  { (free lane_2_near)! (observed lanes) (following) (behind ego_car truck) } # 
  { (QUIT) }
}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}

### Actions
# Look
DecisionRule look {
  X
  { (scene X) (behind  ego_car truck) (observed X)!}
  { (in_sight lane_1_near) (in_sight lane_1_near) (in_sight lane_2_near) (in_sight lane_2_near) (observed X) (following)! komoLook(X)=1. }
}

# Overtake
DecisionRule overtake {
  X
  { (vehicle X) (behind  ego_car X)  (free lane_2_near) }
  { (behind  ego_car X)! (before  ego_car X) komoOvertake(X)=1. }
}

# Wait
DecisionRule follow {
  X
  { (vehicle X) (behind  ego_car X) }
  { (following) komoFollow(X)=1. }
}

### Rules / Observation Model
#Observation model
Rule {
  X
  { (in_sight X)  (NOT_OBSERVABLE free X) }
  { (in_sight X)! (NOT_OBSERVABLE free X)! (free X) }
}


