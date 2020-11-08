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

EVENTUAL_FACTS{ 
{
}
{
(free lane_2)
(observed  lanes)
}
}

BELIEF_START_STATE{ 
{
()=0.05
}
{
()=0.95
}
}

### Termination RULES 
Rule {
  { (before ego_car truck) } # 
  { (QUIT) }
}

Rule {
  { (free lane_2)! (observed lanes) (following) (behind ego_car truck) } # 
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
  { (in_sight lane_2) (observed X) (following)! komoLook(X)=1. }
}

# Overtake
DecisionRule overtake {
  X
  { (vehicle X) (behind  ego_car X)  (free lane_2) }
  { (behind  ego_car X)! (before  ego_car X) komoOvertake(X)=1. }
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


