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
front_free	# 

# keyword
NOT_OBSERVABLE

## constants
observed
following
vehicle
location
scene
ego_car
truck_1
truck_2
lane_2
lanes

## initial state
START_STATE { 
(vehicle ego_car) (vehicle truck_1) (vehicle truck_2) 
(behind  ego_car truck_1)
(scene   lanes)
(following)
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE free lane_2)
(NOT_OBSERVABLE front_free truck_1)
}
{
(NOT_OBSERVABLE free lane_2)
(NOT_OBSERVABLE before truck_2 truck_1)
(NOT_OBSERVABLE front_free truck_2)
}
{
}
}

BELIEF_START_STATE{ 
{
()=0.5
}
{
()=0.3
}
{
()=0.2
}
}

### Termination RULES 
Rule {
  { (before ego_car truck_1) } # 
  { (QUIT) }
}

Rule {
  { (before ego_car truck_2) } # 
  { (QUIT) }
}

Rule {
  { (free lane_2)! (observed lanes) (following) (behind ego_car truck_1) } # 
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
  { (scene X) (behind  ego_car truck_1) (observed X)!}
  { (in_sight lane_2) (in_sight truck_1_front) (observed X) (following)! komoLook(X)=1. }
}

# Overtake
DecisionRule overtake {
  X
  { (vehicle X) (behind  ego_car truck_1) (free lane_2) (front_free X) }
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

Rule {
  X
  { (in_sight X)  (NOT_OBSERVABLE free X)! }
  { (in_sight X)! }
}

Rule {
  X
  { (observed lanes) (NOT_OBSERVABLE front_free X) }
  { (NOT_OBSERVABLE front_free X)! (front_free X) }
}

Rule {
  #X
  { (observed lanes) (NOT_OBSERVABLE before truck_2 truck_1) }
  { (NOT_OBSERVABLE before truck_2 truck_1)! (before truck_2 truck_1) }
}

