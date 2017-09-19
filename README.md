# Baxter2017
This is the repository for the Hkust Summer project "Baxter plays Tic-Tac-Toe while shopping with Simtrack".

For an install guide please see the setup.txt. Make sure to replace the repositories with the forked version present in the HKBaxterteam organisation.

For a more detailed project description see the documentation pdf in the docu folder.

To start the demo make sure you are correctly connected to the Baxter and start the 3 components in separate terminals for better debuging:

Starting the Baxter moveit component:
```
roslaunch game_master baxter_moveit_ttt.launch
```

This starts the Baxter moveit interface as well as the Kinect2bridge.

Next start the Tic-Tac-Toe and Gui components:

```
roslaunch game_master game_prep.launch
```

A Gui will open where the game can be controlled.

Next start the Simtrack component:

```
roslaunch game_master simtrack.launch
```

After a while the output of the simtrack will show in the GUI.

To start the entire demo with only one command use:
```
roslaunch game_master hkust_baxter_demo.launch
```
This starts all the components at the same time.

For how to play and control the game see the documentation.

An example videos can be seen below:

Link 
link 
link 

This project was generated at HKUST in Summer 2017 by Michael C. Welle and Nadine Drollinger, Supervised by Prof. Michael Wang and Prof. Hang Kaiyu



