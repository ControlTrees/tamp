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
holding     # object is held by an agent
hand_empty  # hand is free
on_table    # object X is on the table
on	    # object X is on object Y
clear       # object X top is clear
identified  # object X as been identified, the agnt knows which block it is
is

# keyword
NOT_OBSERVABLE
UNEQUAL

## constants
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
START_STATE { (table tableC) 
(block block_1) (block block_2) (block block_3) (id block_a) (id block_b) (id block_c)
(UNEQUAL block_1 block_2) (UNEQUAL block_1 block_3) (UNEQUAL block_2 block_3)
(location tableC_center) (location tableC_left) (location tableC_right)
(clear block_3) (clear block_2) (clear tableC_right)
(on_table block_1 tableC_center) (on_table block_2 tableC_left) (on block_3 block_1)
(hand_empty) 

(is block_2 block_b)
(identified block_2)
}

EVENTUAL_FACTS{ 
{
(NOT_OBSERVABLE is block_3 block_c)
(NOT_OBSERVABLE is block_1 block_a)
}
{
(NOT_OBSERVABLE is block_1 block_c)
(NOT_OBSERVABLE is block_3 block_a)
}
}

BELIEF_START_STATE{ 
{
()=0.6
}
{
()=0.4
}
}

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
Include = 'LGP-blocks-actions-observations.g'
