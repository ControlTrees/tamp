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
accelerating	# an agent X is accelerating
continuing      # an agent X is continuing
slowing_down    # an agent X is slowing_down

# keyword
NOT_OBSERVABLE
external_agent

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
agent_1

## Declare Agents
__AGENT_0__
__AGENT_1__

## initial state
START_STATE { 
(vehicle ego_car) (vehicle truck) 
(behind  ego_car truck)
(scene   lanes)
(following)
(external_agent agent_1)
}

EVENTUAL_FACTS{ 
{
}
{
(NOT_OBSERVABLE free lane_2)
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
# Agent 0
# Look
DecisionRule __AGENT_0__look {
  X
  { (scene X) (behind  ego_car truck) (observed X)!}
  { (in_sight lane_2) (observed X) (following)! komoLook(X)=1. }
}

# Overtake
DecisionRule __AGENT_0__overtake {
  X
  { (vehicle X) (behind  ego_car X)  (free lane_2) }
  { (behind  ego_car X)! (before  ego_car X) komoOvertake(X)=1. }
}

# Wait
DecisionRule __AGENT_0__follow {
  X
  { (vehicle X) (behind  ego_car X) }
  { (following) komoFollow(X)=1. }
}

# Agent 1
DecisionRule __AGENT_1__accelerate {
  X
  { ( external_agent X ) }
  { ( accelerating X ) ( continuing X )! ( slowing_down X )! komoAccelerate()=1. }
}

DecisionRule __AGENT_1__continue {
  X
  { ( external_agent X ) }
  { ( accelerating X )! ( continuing X ) ( slowing_down X )! komoContinue()=1. }
}

DecisionRule __AGENT_1__slow_down {
  X
  { ( external_agent X ) }
  { ( accelerating X )! ( continuing X )! ( slowing_down X ) komoSlowDown()=1. }
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


