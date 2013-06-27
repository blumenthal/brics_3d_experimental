DESCRIPTION
-----------

Algorithms does essentially table top segmentation. Objects that fit to a certain bounding box volume are 
considered to be "scene objects". Those will be associated with the existin scene objects. The last deteceted
object will define a Region of Interest - which will be taken as a filter for the point cloud input in
the next processing cycle. 

REQUIREMENTS
------------

* OpenNI kinect node
* pcl
* brics_3d
* brics_3d_ros
* brics_3d_msgs
* OpenSceneGraph(OSG) (optional)
	-> to acticate OSG go to the build folder, do a cmake-gui and activate the USE_OSG checkbox
	-> then compile again via make

SAMPLE DATA
-----------

Invoke the get_data.sh script to download a sample data set.


START 
-----

roslaunch brics_3d_icra_2013_demo bring_up_rsg.launch 
