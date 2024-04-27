
.PHONY : test.oct

test.oct :
	python3 scene.py test ../data/scenes/test.scn
	python3 point_cloud.py ../data/scenes/test.scn ../data/ 1000
	python3 octree.py ../data/scenes/test.scn
	

