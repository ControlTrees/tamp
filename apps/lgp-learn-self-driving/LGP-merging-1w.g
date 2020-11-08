Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates

# keyword
NOT_OBSERVABLE

## constants
vehicle
dummy
in_front_of #X, Y X is in front of Y
ego_car
car_1
car_2
car_3
car_4
car_5
car_6
car_7
merged

## initial state
START_STATE { 
(dummy dummy)
(vehicle car_1) (vehicle car_2) (vehicle car_3) (vehicle car_4) (vehicle car_5) (vehicle car_6) (vehicle car_7)
(in_front_of car_2 car_1) (in_front_of car_3 car_2) (in_front_of car_4 car_3) (in_front_of car_5 car_4) (in_front_of car_7 car_6) 
}

EVENTUAL_FACTS{ 
{
}
}

BELIEF_START_STATE{ 
{
()=1.0
}
}

### Termination RULES 
Rule {
  { (merged) } # 
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
# Wait / Approach
DecisionRule continue {
  X
  { (dummy X) }
  { komoContinue(X)=1. }
}

# Select traget
#DecisionRule select {
#  X Y
#  { (vehicle X) (vehicle Y) (in_front_of Y, X) (target_chosen)! }
#  { (car_before X) (car_after Y) (target_chosen) }
#}

# Merge
DecisionRule merge_between {
  X, Y
  { (vehicle X) (vehicle Y) (in_front_of Y X) (merged)! }
  { (merged) (in_front_of ego_car X) (in_front_of Y ego_car) (in_front_of Y X)! komoMergeAfter(X, Y)=1. }
}

