Include = 'data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
#Include = 'data/man_model.ors'
#Include = 'data/pr2_model/pr2_model.ors'

#Edit  waist { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }
#Edit  base { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }

### objs

#right
body  shelf { X=<T t(.9 1 0.93) d(180 0 0 1)> contact }
shape shelf_back(shelf)   { type=9 rel=<T t(0  -0.15 0.6)>   size=[0.7 0.03 1.2 0.005] color=[1 0 0] }
shape shelf_left(shelf)   { type=9 rel=<T t( .35  0.0  0.6)> size=[0.03 0.3 1.2 0.005] color=[1 0 0] }
shape shelf_center(shelf) { type=9 rel=<T t(0 0.0  0.6)>     size=[0.03 0.3 1.2 0.005] color=[1 0 0] }
shape shelf_right(shelf)  { type=9 rel=<T t(-.35 0.0  0.6)>  size=[0.03 0.3 1.2 0.005] color=[1 0 0]  }

shape shelf_bottom_v(shelf) { type=9 rel=<T t(0  0.0 0.0)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }
shape shelf_center_v(shelf) { type=9 rel=<T t(0  0.0 0.6)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }
shape shelf_top_v(shelf)    { type=9 rel=<T t(0  0.0 1.2)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }

#blocks
body block_o { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 1 1] contact }	#on the shelf

#on the shelf

body block_a { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 0 0] contact }
shape id_a(block_a) { type=9 rel=<T t(0.05 0 0.05)> size=[.02 .1 .1 .01] color=[1 0 0] }

body block_b { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 0 0] contact }	
shape id_b(block_b) { type=9 rel=<T t(0.05 0 0.05)> size=[.02 .1 .1 .01] color=[0 1 0] }

body block_c { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 0 0] contact }	
shape id_c(block_c) { type=9 rel=<T t(0.05 0 0.05)> size=[.02 .1 .1 .01] color=[0 0 1] }

#table
body tableC{ type=9, X=<T t(.9 .2 0.9)>, size=[.8 2 .04 .01], color=[.3 .5 .3] contact }

## GRASP references
shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

shape baxterCR (right_wrist){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> contact } #collision model
shape baxterCL (left_wrist){ type=1 size=[0 0 0 0.005] color=[1 1 0]  rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> contact } #collision model

## Joints
joint (shelf block_o)           { from=<T t(-0.225 0 .6  ) t(0 0 0)> to=<T > type=10 } 
joint (tableC block_b)          { from=<T t( 0     -0.3  .02 ) t(0 0 0)> to=<T > type=10 }
joint (tableC block_a)          { from=<T t( 0     -0.15 .02 ) t(0 0 0)> to=<T > type=10 } 
joint (block_a block_c)         { from=<T t( 0.0   0    .1  ) t(0 0 0)> to=<T > type=10 } 
