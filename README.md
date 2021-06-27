# ROS_Feedback_controllers
Fuzzy logic and PID controllers for multiple robot behaviors

Closed loop PID and fuzzy logic controllers for right edge following and obstacle avoidance using laser sesnor. 
ROS along with python was used for controller development. 
Individual classes was developed and they are tested in Gazebo simulator.
## Fuzzy Right Edge Following controller
### Input Membership function
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_MF.PNG)
### Rule Base
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_RuleBase.PNG)
### Output Membership Function
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_OMF.PNG)
### Control surface 
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/CS_REF_Str.PNG)
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_FS_CS.PNG)

## Fuzzy Obstacle avoidance function
### Input Membership function 
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_IMF.PNG)
### Rule Base
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_Rule_base.PNG)
### Output Membership Function
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_OMF.PNG)
### Control Surface
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_FS_CS.PNG)
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_ST_CS.PNG)

## Fuzzy Coordination
Individual behaviours was merged using fuzzy coordination.
### Bahaviour selection Logic
(https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/Fuzzy_Coordination.PNG)
