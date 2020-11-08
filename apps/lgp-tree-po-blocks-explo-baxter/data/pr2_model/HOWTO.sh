cp $1 z.urdf
../../bin/urdf2ors.py > z1-raw.ors
sed 's/package:\/\/pr2_description\/meshes\///g' z1-raw.ors > z2-paths.ors
cat pr2_before.ors z2-paths.ors pr2_after.ors > z3-augmented.ors
../../bin/ors_editor -file z3-augmented.ors -cleanOnly
mv z.ors z4-clean.ors

