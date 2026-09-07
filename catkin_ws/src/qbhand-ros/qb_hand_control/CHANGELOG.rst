^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qb_hand_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.3 (2023-04-26)
------------------
* FEAT: added parameters to specify the serial port to connect to.

3.0.2 (2022-09-20)
------------------

3.0.1 (2022-07-07)
------------------
* Added control for qbSotHand2 chain.

3.0.0 (2022-07-06)
------------------
* Removed some comments
* Implemented a new controller to control SH2R motors independently

2.3.0 (2022-04-08)
------------------
* modified SHR2 waypoints and limit range
* modified urdf and launch in order to attach SHR2 to robots urdf

2.2.3 (2021-11-05)
------------------
* Minor fix on c++ version and parameters in launch files.

2.2.2 (2021-08-30)
------------------

2.2.1 (2021-08-27)
------------------

2.2.0 (2021-08-27)
------------------
* Add Gazebo SoftHand and SoftHand 2 Motors plugin skeletons
* Add hardware interface for SoftHand 2 Motors
* Set a better name for fake joint simulation variables
* Fix cmake resource installation
* Fix launch
* Disable Gazebo support for qb SoftHand
* Added tools(launch and URDF) to control more than one hand.

2.1.2 (2019-05-28)
------------------
* Fix minor style issues

2.1.1 (2018-08-09)
------------------

2.1.0 (2018-08-07)
------------------
* Set safer default values

2.0.0 (2018-05-30)
------------------
* Update README
* Refactor launch files
* Remove control node
* Add blocking setCommands method support
* Refactor launch files
* Fix destructor calls on ROS shutdown

1.0.6 (2017-11-24)
------------------

1.0.5 (2017-06-27)
------------------
* Fix C++11 support for cmake version less than 3.1

1.0.4 (2017-06-23)
------------------
* Update cmake version to match Kinetic standards

1.0.3 (2017-06-22)
------------------
* fix dependencies

1.0.2 (2017-06-21)
------------------
* fix cmake settings to solve isolated builds

1.0.1 (2017-06-19)
------------------
* first public release for Kinetic
