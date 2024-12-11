# Introduction

Hide and seek game consists of two separate teams; Hiders and Seekers. Hider's main objective is to find the best hiding spot that will prevent them from being found for the longest time. There is no limit to using sensory devices but there is a limit to actuators, specifically motor speed. There also is no limit to using any ROS topics. Same rules apply for Seekers. The Seeker's main objective is to find the Hiders. To better detect Hiders, Hiders will have fiducials attached on all four sides of the robot. This imposes one additional mandatory task for seekers which is having to seek for fiducials at all times.


# Summary
## James
## Daphne
## Chloe
I designed a hiding algorithm that identifies areas that are walled in on three sides - pockets - using lidar, and navigates to them using move_base. My project runs using *hider_real.py*, *scan_sim.py*, *timer.py*, and *my_odom.py*. The identification of pockets and coordinate calculation is done in *scan_sim.py* and move_base is handled in *hider_real.py*. *my_odom.py* and *timer.py* are helper programs.

In order to better develop my pocket detection program, I graphed the lidar data in the spreadsheet below. The spreadsheet also includes notes on the debugging process and the success of the tests.

Data and progression: [https://docs.google.com/spreadsheets/d/1lpmWlnTqMWZSKndbC97cu0bUynrpZY1xOJgwtPlUaSQ/edit?usp=sharing]


# Video
# Lab Notebook project report
