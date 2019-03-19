# knowrob_robcog #
## Setup
### Windows
Follow RobCoG isntallation instructions [here](https://github.com/robcog-iai/RobCoG)
In the RobCoG plugins folder run:

`git clone https://github.com/robcog-iai/UROSWorldControl.git`

`git clone https://github.com/robcog-iai/UROSBridge.git`


### ROS
In ROS all of the following packages need to be downloaded and build, if needed please follow the instructions of the individual packages:


 * [knowrob](http://www.knowrob.org/installation)
 * [unreal_ros_pkgs](https://github.com/robcog-iai/unreal_ros_pkgs) (NOTE: This needs to be build seperatly before the knowrob_robcog package!)
 * [knowrob_robcog](https://github.com/robcog-iai/knowrob_robcog/tree/sem-map-cpp)
 * [rosbridge_suite](http://wiki.ros.org/rosbridge_suite)

 Once all the everything is build the rosbridge_suite websocket can be run by using the following command:
 
 ` roslaunch rosbridge_server rosbridge_websocket.launch `


## Usage
#### Unreal
The Unral RobCoG Project needs to be loaded in the Unreal Editor, with an empty/new level Loaded.
For the highlighting services to work a PostProcessVolume needs to be placed in the world, the UROSWorldControl plugin provides an empty level with the PostProcessVolume set up correctly, it can be found in the contents folder and has the name *Empty_with_PostProcessVolume*. If this it is placed by hand two things need to be set up, the *Infinite Extent (Ubound)* option needs to be enabled and the *PP_Outliner_M_Inst* needs to be added to the *Post Process Materials*.

To connect the Unreal Editor with the rosbridge websocket the UROSWorldControl Mode-Tab in the editor can be used. Simply set the *Server Adress* accordingly, add the RWCManager to the *Publisher List* and click the connect button.
If everything is set up correctly the Output Log should show all the services that have been published and the *Connectino Status* should change to *Connected to Rosbridge.*

### ROS
To execute the knowrob_robcog package run:

` rosrun rosprolog rosprolog knowrob_robcog `

The rosprolog prolog console should be startet now.  Before the a semantic map can be spawned it needs to be parse, which can be done by running:

` owl_parse('PATH'). ` T

The path can be relative, so starting the prolog consol in same folder the semantic map is safed at can be helpful.

Parsing of the the semantic map can also be added to the *init.pl* file, by uncommenting line 56 and changing the path accordingly.

Either way we can see if the map was parsed correctly by running:

` map_instance(Map). `

Once the semantic map is parsed it can be spawned by running:

` spawn_semantic_map(Map) `

NOTE: This will work if only one semantic map is loaded in the environment, otherwise the variable Map needs to point to the exact map instace that should be spawned.

After the map is spawned the following prolog calls can be used to highlight objects in the enivornment.

 * get_subclass_in_map(Map, ClassType, Individual)
 * highlight(ObjectIndividual)
 * highlight_everything_that_is(Map, ObjectType)
 * highlight_likely_storage_place(Map, ObjectTypeToBeStored)
 * highlight_handle_of(Map, ObjectIndividual, HandleIndividual)
 * highlight_device_for_action(Map, Action, Device)

 Some example calls can be seen below:

 ```

get_subclass_in_map(Map, 'http://knowrob.org/kb/knoworb.owl#Refriguator', Individual),
highlight(Individual).

highlight_everything_that_is(Map, 'http://knowrob.org/kb/knoworb.owl#Handle').

highlight_likely_storage_place(Map, 'http://knowrob.org/kb/knoworb.owl#Perishable').

get_subclass_in_map(Map, 'http://knowrob.org/kb/knoworb.owl#Oven', Individual),
highlight_handle_of(Map, Individual, Handle).

highlight_device_for_action(Map, 'http://knowrob.org/kb/knoworb.owl#WashingDishesByMachine').

 ```


