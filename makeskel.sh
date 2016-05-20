perl ./cleanskel.pl *.cpp *.h
perl ./skelrenamer.pl *.skel
mv Makefile.skel Makefile
mv Animator.vcproj.skel Animator.vcproj
rm *.skel
rm antigravity.* beziercurveevaluator.* bsplinecurveevaluator.* \
	   c2icurveevaluator.* crcurveevaluator.* drag.*  floor.* forceField.h\
	   force.h gravity.* particle.* pointObj.* wind.*
