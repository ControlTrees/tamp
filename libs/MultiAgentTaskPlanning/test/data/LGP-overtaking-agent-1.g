

BELIEF_START_STATE{ 
{
(free lane_2)
()=0.05
}
{
#(free lane_2)
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


