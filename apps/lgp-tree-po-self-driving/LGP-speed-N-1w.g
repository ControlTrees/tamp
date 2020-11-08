Include = 'data/keywords.g'

FOL_World{
  hasWait=false
  gamma = 1.
  stepCost = 1.
  timeCost = 0.
}

## basic predicates

# keyword

## constants
vehicle
at_speed0
at_speed1
at_speed2
at_speed3
at_speed4
at_speed5
at_speed6
at_speed7
at_speed8
at_speed9
at_speed10

## initial state
START_STATE { 
(vehicle at_speed0)
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
  { (vehicle at_speed3) } # 
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
# Speed1
DecisionRule speed1 {
  X
  { (X at_speed0)}
  { (X at_speed1) (X at_speed0)! komoSpeed1(X)=1. }
}

# Speed2
DecisionRule speed2 {
  X
  { (X at_speed1)}
  { (X at_speed2) (X at_speed1)! komoSpeed2(X)=1. }
}

# Speed3
DecisionRule speed3 {
  X
  { (X at_speed2)}
  { (X at_speed3) (X at_speed2)! komoSpeed3(X)=1. }
}

# Speed4
DecisionRule speed4 {
  X
  { (X at_speed3) }
  { (X at_speed4) (X at_speed3)! komoSpeed4(X)=1. }
}

# Speed5
DecisionRule speed5 {
  X
  { (X at_speed4)}
  { (X at_speed5) (X at_speed4)! komoSpeed5(X)=1. }
}

# Speed6
DecisionRule speed6 {
  X
  { (X at_speed5)}
  { (X at_speed6) (X at_speed5)! komoSpeed6(X)=1. }
}

# Speed7
DecisionRule speed7 {
  X
  { (X at_speed6)}
  { (X at_speed7) (X at_speed6)! komoSpeed7(X)=1. }
}


# Speed8
DecisionRule speed8 {
  X
  { (X at_speed7)}
  { (X at_speed8) (X at_speed7)! komoSpeed8(X)=1. }
}

# Speed9
DecisionRule speed9 {
  X
  { (X at_speed8)}
  { (X at_speed9) (X at_speed8)! komoSpeed9(X)=1. }
}

# Speed10
DecisionRule speed10 {
  X
  { (X at_speed9)}
  { (X at_speed10) (X at_speed9)! komoSpeed10(X)=1. }
}

