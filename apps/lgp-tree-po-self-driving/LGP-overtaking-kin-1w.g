#Include = 'data/man_model.ors'

### objs

#vehicles
body car_ego { type=9 rel=<T t(0 0.0 0.05)> size=[.2 .1 .1 .003]  color=[0 1 1] contact }
body car_op  { type=9 rel=<T t(0 0.0 0.05)> size=[.2 .1 .1 .003]  color=[1 0 0] contact type=1 }
#body car_op_2  { type=9 rel=<T t(0 0.0 0.05)> size=[.2 .1 .1 .002]  color=[1 0 0] }
body truck   { type=9 rel=<T t(0 0.0 0.05)> size=[.5 .12 .2 .05]  color=[1 0 1] contact type=1 }

#road
body lane_1{ type=9, X=<T t(0.0 -0.10 0.0)>, size=[5. .2 .04 .01], color=[.3 .3 .3] }
body line  { type=9, X=<T t(0.0 0.0 0.0)>,   size=[5. .02  .041 .01],color=[1   1  1] }
body lane_2{ type=9, X=<T t(0.0  0.10 0.0)>, size=[5. .2 .04 .01], color=[.3 .3 .3] }

## Joints
joint (lane_1 car_ego)         { from=<T t( -2.0   0.01 .05 ) t(0 0 0)> to=<T > type=6 agent=true}
joint (lane_1 truck)           { from=<T t( -1.3   0 .05 ) t(0 0 0)> to=<T > type=3 }
joint (lane_2 car_op)          { from=<T t( 1.7      0 .05 ) t(0 0 0)> to=<T > type=3 }
#joint (lane_2 car_op_2)          { from=<T t( -1.0      0 .05 ) t(0 0 0)> to=<T > type=6 }
#joint (tableC block_a)          { from=<T t( 0.15  0 .02 ) t(0 0 0)> to=<T > type=10 } 
#joint (block_a block_c)         { from=<T t( 0.0   0 .1  ) t(0 0 0)> to=<T > type=10 } 
