# Check
DecisionRule check {
  X, Y
  { (block X) (side Y) (holding X) (identified X)! }
  { (in_sight Y) komoCheck(X, Y)=1. }
}

# Pick-up
DecisionRule pick-up {
  X, Y
  { (block X) (location Y) (on_table X Y) (hand_empty) }
  { (on_table X Y)! (holding X) (hand_empty)! komoPickUp(X Y)=1. }
}

# Put-down
DecisionRule put-down {
  X, Y
  { (block X) (location Y) (holding X) (identified X) }
  { (holding X)! (hand_empty) (on_table X Y) komoPutDown(X Y)=1. }
}

### Rules / Observation Model
#Observation model
Rule {
  X, Y
  { (block X) (side Y) (NOT_OBSERVABLE colored Y) (in_sight Y) }
  { (in_sight Y)! (colored Y) (NOT_OBSERVABLE colored Y)! (identified X)}
}

