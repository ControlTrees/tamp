Include = 'data/baxter_model/baxter.ors'
Delete shape visual
#Delete shape collision

Include = 'data/man_model.ors'


Edit  waist { X=<T t(1.5 0. 1.) d(-90 0 0 1)> }

body tableC{ type=9, X=<T t(.7 0 .8)>, size=[1. .8 .04 .02], color=[.3 .3 .5] fixed, contact }
body tableL{ type=9, X=<T t(.2 .7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }
body tableR{ type=9, X=<T t(.2 -.7 .8)>, size=[2. .6 .04 .02], color=[.3 .5 .3] fixed, contact }

#body obj1 { size=[.06 .06 .1 .02] type=9 contact }
#joint (tableC obj1) { from=<T t(-.2 .2 0)> to=<T t(0 0 .1)> type=10 }
#shape shape1 (obj1) { type=5 rel=<T t(0 0 .1)>size=[.4 0 0 0] }

#body Handle { type=9 size=[.03 .3 .15 .02] contact }
#joint (tableC Handle) { from=<T t(-.2 .2 0)> to=<T t(0 0 .1)> type=10 }

#body Long1 { type=9 size=[.03 .3 .1 .02] contact }
#joint (tableC Long1) { from=<T t(.2 .2 0)> to=<T t(0 0 .1)> type=10 }

#body Long2 { type=9 size=[.03 .3 .1 .02] contact }
#joint (tableC Long2) { from=<T t(.2 -.2 0)> to=<T t(0 0 .1)> type=10 }

#shape aLong2 (Long1) { type=5 }

#shape A1(left_wrist){ rel=<T t(.24 0 0)> type=5 color=[1 0 0] size=[.2 0 0 0]}
#shape A2(right_wrist){ rel=<T t(.24 0 0)> type=5 color=[1 0 0] size=[.2 0 0 0]}
#shape A3(handR){ type=5 color=[1 0 0] size=[.2 0 0 0]}


## GRASP references

#body humanGraspRefR { type=5 size=[.1 0 0 0] color=[1 0 0] }
#body humanGraspRefL { type=5 size=[.1 0 0 0] color=[1 0 0] }
#joint humanGraspJointR(handR humanGraspRefR){ A=<T t(0 0 -.1)> type=11 ctrl_H=1e-4 }
#joint humanGraspJointL(handL humanGraspRefL){ A=<T t(0 0 -.1)> type=11 ctrl_H=1e-4 }
shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

shape baxterR (right_wrist){ rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }
shape baxterL (left_wrist) { rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> type=5 size=[.1 0 0 0] color=[1 1 0] }

#body baxterGraspRefR { type=5 size=[.5 0 0 0] color=[1 0 0] }
#body baxterGraspRefL { type=5 size=[.1 0 0 0] color=[1 0 0] }
#joint baxterGraspJointR(right_wrist baxterGraspRefR){ A=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.24)> type=11 ctrl_H=1e1 }
#joint baxterGraspJointR(left_wrist baxterGraspRefL){ A=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.24)> type=11 ctrl_H=1e1 }
#shape baxterGraspRefR (right_wrist){ type=5 size=[.1 0 0 0] color=[1 0 0] rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> }
#shape baxterGraspRefL (left_wrist) { type=5 size=[.1 0 0 0] color=[1 0 0] rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26)> }


Include = 'data/toolbox/toolbox.ors'

joint (tableC /toolbox/handle) { from=<T t(0 0 .04) t(0 0 .12)> to=<T > type=10 }
joint (tableC /toolbox/side_front) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(-90 0 0 1) t(0 -.212 0)> type=10 }
joint (tableC /toolbox/side_back) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(90 0 0 1) t(0. -.212 0)> type=10 }
joint (tableC /toolbox/side_left) { from=<T t(0 0 .04) t(0 0 .12)> to=<T t(.0 -.147 .0)> type=10 }
joint (tableC /toolbox/side_right) { from=<T t(0 0 .04) t(0 0 .12)> to=<T d(180 0 0 1) t(.0 -.147 0)> type=10 }
joint (tableC /toolbox/floor_left) { from=<T t(0 0 .04)> to=<T t(.0 .069 .004) d(90 1 0 0)> type=10 }
joint (tableC /toolbox/floor_right) { from=<T t(0 0 .04)> to=<T d(180 0 0 1) t(.0 .069 .004) d(90 1 0 0)> type=10 }

Include = 'data/screwdriver/screwdriver.ors'
joint (tableR screwdriver) { from=<T t(0 0 .06)> to=<T t(-.5 .0 .0) > type=10 }
#joint (tableR screwdriver) { from=<T t(0 0 .06)> to=<T t(.5 .0 .0) > type=10 }
shape screwdriverHandle(screwdriver) { type=5 rel=<T d(90 0 0 1)> size=[.15 0 0 0] color=[1 1 0] }

body screwbox { type=9, size=[.05 .1 .04 .02], color=[.8 .3 .3] fixed, contact }
joint (tableL screwbox) { from=<T t(.8 0 .08)> type=10 }
