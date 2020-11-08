Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates
#location
#object
#mutex_objects	# objects that should never collide
#container
block
location
table
id	    # block identifier

in_sight   # block identification part is visible
#view_taken # object recognition has been triggered for checking if the object X is at location Y
#viewed	   # object X has been viewed at location Y
#at         # object X is at location Y
#grasped    # agent X holds/has grasped object Y
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear
identified  # object X as been identified, the agnt knows which block it is
is

#now_in_sight # getting sight of location X
#collision_avoidance_activated # 

# keyword
NOT_OBSERVABLE

## constants
#container_0
#container_1
block_1
block_2 
block_3 
block_a  #block identifier
block_b  #block identifier
block_c  #block identifier
tableC
tableC_center
tableC_left
tableC_right


## initial state
START_STATE { 
(table tableC) 
(location tableC_center) (location tableC_left) (location tableC_right)
(block block_1) (block block_2) (block block_3)
(id block_a) (id block_b) (id block_c)
(clear block_3) (clear block_2) (clear tableC_right)
(on_table block_1 tableC_center) (on_table block_2 tableC_left)
(on block_3 block_1)
(hand_empty) 
(is block_2 block_b)
(identified block_2)
}

BELIEF_START_STATE{ 
{
(is block_3 block_c)
(is block_1 block_a)
()=0.6
}
{
(is block_1 block_c)
(is block_3 block_a)
()=0.4
}
}

### Termination RULES 
Rule {
  { (on block_a block_b) (on block_b block_c) (hand_empty) } # 
  { (QUIT) }
}

Rule {
  { (identified block_3) } # 
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
# Check
DecisionRule check {
  X
  { (block X) (holding X) (identified X)! }
  { (in_sight X) komoCheck(X)=1. }
}

# Pick-up
DecisionRule pick-up {
  X, Y
  { (block X) (location Y) (clear X) (on_table X Y) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoPickUp(X Y)=1. }
}

# Unstack
DecisionRule unstack {
  X, Y
  { (block X) (block Y) (clear X) (on X Y) (hand_empty) }
  { (on X Y)! (holding X) (hand_empty)! (clear X)! (clear Y) komoUnStack(X Y)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y
  { (block X) (location Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear X)  (clear Y)! (on_table X Y) komoPutDown(X Y)=1. }
}

# Stack
DecisionRule stack {
  X, Y
  { (block X) (block Y) (holding X) (clear Y) }
  { (holding X)! (hand_empty) (clear X) (clear Y)! (on X Y) komoStack(X Y)=1. }
}

### Rules / Observation Model
#Observation model
Rule {
  X, Y
  { (block X) (id Y) (NOT_OBSERVABLE is X Y) (in_sight X) }
  { (in_sight X)! (is X Y) (identified X) (NOT_OBSERVABLE is X Y)! }
}

#deduction of the last block if they have all been identified..(is it rigorous?)
Rule {
  X, Y, Z, T
  { (block X) (block Y) (block Z) (id T) (identified X) (identified Y) (identified Z)! (NOT_OBSERVABLE is Z T)}
  { (identified Z) (is Z T) (NOT_OBSERVABLE is Z T)!}
}

#Apply identification to the ON
Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y) }
  { (on Z T)}
}

Rule {
  X, Y, Z, T
  { (block X) (block Y) (id Z) (id T) (is X Z) (is Y T) (on X Y)! (on Z T)}
  { (on Z T)!}
}

#Apply identification to the ON table
Rule {
  X, Y, Z
  { (block X) (location Y) (id Z) (on_table X Y) (is X Z) }
  { (on_table Z Y)}
}

Rule {
  X, Y, Z
  { (block X) (location Y) (id Z) (on_table X Y)! (on_table Z Y) (is X Z) }
  { (on_table Z Y)!}
}

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



