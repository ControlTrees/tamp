Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## activities
#get_sight
#take_view
pick-up
place

## basic predicates
#location
#object
#mutex_objects	# objects that should never collide
#container
block
location
table

#in_sight   # location X is visible
#view_taken # object recognition has been triggered for checking if the object X is at location Y
#viewed	   # object X has been viewed at location Y
#at         # object X is at location Y
#grasped    # agent X holds/has grasped object Y
holding     # object is held by an agent
hand_empty   # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear

#now_in_sight # getting sight of location X
#collision_avoidance_activated # 

# keyword
PRE_QUIT
#NOT_OBSERVABLE

## constants
#container_0
#container_1
block_a
block_b
block_c
tableC
tableC_center
tableC_left
tableC_right


## initial state
START_STATE { (table tableC) (block block_a) (block block_b) (block block_c)
(location tableC_center) (location tableC_left) (location tableC_right)
(clear block_b) (clear block_c) (clear tableC_right)
(on_table block_a tableC_center) (on_table block_b tableC_left) (on block_c block_a)
(hand_empty) }
#BELIEF_START_STATE{ (at target container_0)=0.8 (at target container_1)=0.2 }

### Termination RULES 
Rule {
  { (on block_a block_b) (on block_b block_c) (hand_empty) } # 
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
# Pick-up
DecisionRule pick-up {
  X, Y
  { (block X) (location Y) (clear X) (on_table X Y) (hand_empty) }
  { (on_table X Y)! (clear X)! (hand_empty)! (holding X) (clear Y) komoPickUp(X Y)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y
  { (block X) (location Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear X) (on_table X Y) (clear Y)! komoPutDown(X Y)=1. }
}

# Stack
DecisionRule stack {
  X, Y
  { (block X) (block Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear Y)! (clear X) (on X Y) komoStack(X Y)=1. }
}

# Unstack
DecisionRule unstack {
  X, Y
  { (block X) (block Y) (on X Y) (clear X) (hand_empty) }
  { (holding X) (hand_empty)! (clear X)! (clear Y) (on X Y)! komoUnStack(X Y)=1. }
}

# Get sight
#DecisionRule get_sight {
#  X
#  { (INFEASIBLE get_sight X)! (location X) (in_sight X)! } #(in_sight X)! is a trick
#  { (now_in_sight X) komoGetSight(X)=1. }
#}

# Take a view
#DecisionRule take_view {
#  X, Y
#  { (object X) (in_sight Y) (view_taken X Y)! }
#  { (view_taken X Y) komoTakeView(X Y)=1.0 }
#}

# Grasp
#DecisionRule grasp {
#  X, Y
#  { (grasped X Y)! (INFEASIBLE grasp X Y)! (agent X) (container Y) (held Y)! (busy X)! }
#  { (grasped X Y)=2.0 (busy X) (held Y) (on_table Y)! komoGrasp(X Y)=1. }
#}

# Place
#DecisionRule place {
#  X, Y, Z,
#  { (placed X Y Z)! (grasped X Y) (table Z) }
#  { (placed X Y Z)  (on_table Y)  (busy X)! (held Y)! komoPlace(X Y Z)=1. }
#}

#DecisionRule home {
#  { (PRE_QUIT) }
#  { (QUIT) }
#}


### Rules / Observation Model
# Object seen = Observation model
#Rule {
#  X, Y
#  { (object X) (location Y) (NOT_OBSERVABLE at X Y) (in_sight Y) (view_taken X Y) }
#  { (viewed X Y)  (at X Y)  (NOT_OBSERVABLE at X Y)!}
#}

# remove old in sights
#Rule {
#  X, Y
#  { (now_in_sight X) (in_sight Y) }
#  { (in_sight Y)! }
#}

# transform now in sight in in sight
#Rule {
#  X
#  { (now_in_sight X) }
#  { (now_in_sight X)! (in_sight X) }
#}

### Collision avoidance
# over plane activation
#Rule {
#  X, Y
#  { (container X) (table Y) (on_table X)! (collision_avoidance_activated X Y)! }
#  { (collision_avoidance_activated X Y) komoActivateOverPlane(X Y)=1. }
#}

# over plane deactivation
#Rule {
#  X, Y
#  { (container X) (table Y) (on_table X) (collision_avoidance_activated X Y) }
#  { (collision_avoidance_activated X Y)! komoDeactivateOverPlane(X Y)=1. }
#}

# mutex collision avoidance
#Rule {
#  X, Y
#  { (mutex_objects X) (mutex_objects Y) (collision_avoidance_activated X Y)! }
#  { (collision_avoidance_activated X Y) komoCollisionAvoidance(X Y)=1. }
#}


