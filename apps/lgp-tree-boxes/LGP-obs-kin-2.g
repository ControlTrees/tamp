#Include = '../../data/baxter_model/baxter.ors'
#Delete shape visual

#Include = '../../data/reba-human/human.ors'
Include = 'data/man_model.ors'

### objs

#body  target { X=<T t(.0 -1.0 1.9)> }
#shape target(target) { type=1 size=[0 0 0 .05] color=[1 0 0] }

body occluding_object { X=<T t(.0 -0.6 1.9)> }
shape occluding_object(occluding_object) { type=9 size=[0.4 0.01 0.4 0.02] color=[1 0 0] }

body occluding_object_2 { X=<T t(-0.6 0 1.9)> }
shape occluding_object_2(occluding_object_2) { type=9 size=[0.01 0.4 0.4 0.02] color=[1 0 0] }

#body  target_2 { X=<T t(-1.0 0.0 1.9)> }
#shape target_2(target_2) { type=1 size=[0 0 0 .05] color=[1 0 0] }

shape humanR (handR){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }
shape humanL (handL){ type=5 size=[.1 0 0 0] color=[1 1 0] rel=<T t(0 0 -.05) d(90 0 0 1)> }

BELIEF_START_STATE{ 
	{ body  target { X=<T t(.0 -1.0 1.9)> }
	  shape target(target) { type=1 size=[0 0 0 .05] color=[1 0 0] } }

	{ body  target { X=<T t(-1.0 0.0 1.9)> }
	  shape target(target) { type=1 size=[0 0 0 .05] color=[0 1 0] } }
}
