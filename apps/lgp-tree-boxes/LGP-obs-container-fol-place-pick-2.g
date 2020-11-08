Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## activities
get_sight
take_view
grasp
place

## basic predicates
agent
location
object
mutex_objects	# objects that should never collide
container
table

in_sight   # location X is visible
view_taken # object recognition has been triggered for checking if the object X is at location Y
viewed	   # object X has been viewed at location Y
at         # object X is at location Y
grasped    # agent X holds/has grasped object Y
placed
held       # object is held by an agent
busy       # agent X ist involved in an ongoing (durative) activity
on_table   # object X is on the table

now_in_sight # getting sight of location X
collision_avoidance_activated # 

# keyword
PRE_QUIT
NOT_OBSERVABLE

## constants
container_0
container_1
target
handL
handR
tableC
#tableC

## initial state
START_STATE { (agent handL) (mutex_objects handR) (mutex_objects handL) (mutex_objects tableC) (container container_1) (container container_0) (container container_1) (object target) (table tableC) (on_table container_0) (on_table container_1) (location container_0) (location container_1)} #(agent handR)
BELIEF_START_STATE{ (at target container_0)=0.8 (at target container_1)=0.2 }

### Termination RULES 
#Rule {
#  { (view_taken target target_location_1) (view_taken target target_location_2)} # 
#  { (QUIT) }
#}

Rule {
  { (viewed target container_0) (grasped handR target)  (on_table container_0) (on_table container_1) } # 
  { (QUIT) }
}

Rule {
  { (viewed target container_1) (grasped handR target) (on_table container_0) (on_table container_1) } #
  { (QUIT) }
}

Rule {
  { (viewed target container_0) (grasped handL target) (on_table container_0) (on_table container_1) } # 
  { (QUIT) }
}

Rule {
  { (viewed target container_1) (grasped handL target) (on_table container_0) (on_table container_1) } #
  { (QUIT) }
}

### Reward
REWARD {
#  tree{
#    leaf{ { (grasped handR screwdriverHandle) }, r=10. }
#    weight=1.
#  }
}

### Tasks definitions
# Get sight
DecisionRule get_sight {
  X
  { (INFEASIBLE get_sight X)! (location X) (in_sight X)! } #(in_sight X)! is a trick
  { (now_in_sight X) komoGetSight(X)=1. }
}

# Take a view
DecisionRule take_view {
  X, Y
  { (object X) (in_sight Y) (view_taken X Y)! }
  { (view_taken X Y) komoTakeView(X Y)=1.0 }
}

# Grasp
DecisionRule grasp {
  X, Y
  { (grasped X Y)! (INFEASIBLE grasp X Y)! (agent X) (container Y) (held Y)! (busy X)! }
  { (grasped X Y)=2.0 (busy X) (held Y) (on_table Y)! komoGrasp(X Y)=1. }
}

DecisionRule graspObject {
  X, Y, Z
  { (grasped X Y)! (INFEASIBLE grasp X Y)! (agent X) (held Y)! (busy X)! (location Z) (viewed Y Z) }
  { (grasped X Y)=2.0 (busy X) (held Y) komoGraspObject(X Y)=1. }
}

# Place
DecisionRule place {
  X, Y, Z,
  { (placed X Y Z)! (grasped X Y) (table Z) }
  { (placed X Y Z)  (on_table Y)  (busy X)! (held Y)! komoPlace(X Y Z)=1. }
}

#DecisionRule home {
#  { (PRE_QUIT) }
#  { (QUIT) }
#}


### Rules / Observation Model
# Object seen = Observation model
Rule {
  X, Y
  { (object X) (location Y) (NOT_OBSERVABLE at X Y) (in_sight Y) (view_taken X Y) }
  { (viewed X Y)  (at X Y)  (NOT_OBSERVABLE at X Y)!}
}

# remove old in sights
Rule {
  X, Y
  { (now_in_sight X) (in_sight Y) }
  { (in_sight Y)! }
}

# transform now in sight in in sight
Rule {
  X
  { (now_in_sight X) }
  { (now_in_sight X)! (in_sight X) }
}

### Collision avoidance
# over plane activation
Rule {
  X, Y
  { (container X) (table Y) (on_table X)! (collision_avoidance_activated X Y)! }
  { (collision_avoidance_activated X Y) komoActivateOverPlane(X Y)=1. }
}

# over plane deactivation
#Rule {
#  X, Y
#  { (container X) (table Y) (on_table X) (collision_avoidance_activated X Y) }
#  { (collision_avoidance_activated X Y)! komoDeactivateOverPlane(X Y)=1. }
#}

# mutex collision avoidance
Rule {
  X, Y
  { (mutex_objects X) (mutex_objects Y) (collision_avoidance_activated X Y)! }
  { (collision_avoidance_activated X Y) komoCollisionAvoidance(X Y)=1. }
}


