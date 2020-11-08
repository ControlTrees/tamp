#Include = '../../data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
Include = 'data/man_model.ors'
#Include = 'data/pr2_model/pr2_model.ors'

### objs

#right
body  shelf { X=<T t(-.75 -0.7 0.93) d(-90 0 0 1)> }
shape shelf_back(shelf)   { type=9 rel=<T t(0  -0.15 0.6)>   size=[0.7 0.03 1.2 0.005] color=[1 0 0] }
shape shelf_left(shelf)   { type=9 rel=<T t( .35  0.0  0.6)> size=[0.03 0.3 1.2 0.005] color=[1 0 0] }
shape shelf_center(shelf) { type=9 rel=<T t(0 0.0  0.6)>     size=[0.03 0.3 1.2 0.005] color=[1 0 0] }
shape shelf_right(shelf)  { type=9 rel=<T t(-.35 0.0  0.6)> size=[0.03 0.3 1.2 0.005]  color=[1 0 0]  }

shape shelf_bottom_v(shelf) { type=9 rel=<T t(0  0.0 0.0)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }
shape shelf_center_v(shelf) { type=9 rel=<T t(0  0.0 0.6)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }
shape shelf_top_v(shelf)    { type=9 rel=<T t(0  0.0 1.2)> size=[0.7 0.3 0.01 0.005]   color=[1 0 0] }

#blocks
body block_o { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .02] color=[0 1 1] }	#on the shelf

body block_1 { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 0 0] }	
body block_2 { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 1 0] }	
body block_3 { type=9 rel=<T t(0 0.0 0.05)> size=[.1 .1 .1 .01] color=[0 0 0] }

#table
body tableC{ type=9, X=<T t(-0.3 -.7 0.9)>, size=[2. .8 .04 .01], color=[.3 .5 .3] contact }
shape tableC_center(tableC){ type=9, rel=<T t(0.25 0 0.01)>, size=[0.5 .8 .04 .01],  color=[.5 .5 .3] }
shape tableC_left(tableC)  { type=9, rel=<T t( 0.75 0 0.01)>, size=[0.5 .8 .04 .01], color=[.3 .5 .3] }
shape tableC_right(tableC) { type=9, rel=<T t(-0.25 0 0.01)>, size=[0.5 .8 .04 .01], color=[.3 .5 .5] }

## GRASP references
shape humanR (handR){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0 0 0) d(0 0 0 1)> contact }
shape humanL (handL){ type=1 size=[0 0 0 0.005] color=[1 1 0] rel=<T t(0 0 0) d(0 0 0 1)> contact }

## Ids
#shape id_1(block_1) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[1 0 0] }
shape id_2(block_2) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[0 1 0] }
#shape id_3(block_3) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[0 0 1] }

## Joints
joint (shelf block_o)           { from=<T t(-0.225 0 .6  ) t(0 0 0)> to=<T > type=10 } 
joint (tableC block_2)          { from=<T t( 0.3   0 .02 ) t(0 0 0)> to=<T > type=10 }
joint (tableC block_1)          { from=<T t( 0.15  0 .02 ) t(0 0 0)> to=<T > type=10 } 
joint (block_1 block_3)         { from=<T t( 0.0   0 .1  ) t(0 0 0)> to=<T > type=10 } 

BELIEF_START_STATE { 
{
 shape id_1(block_1) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[1 0 0] }
 shape id_3(block_3) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[0 0 1] }
}
{
 shape id_1(block_1) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[0 0 1] }
 shape id_3(block_3) { type=9 rel=<T t(0 -0.05 0.05)> size=[.1 .02 .1 .01] color=[1 0 0] }
}
}
