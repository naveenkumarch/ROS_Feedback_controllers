# ROS_Feedback_controllers
Fuzzy logic and PID controllers for multiple robot behaviors

Closed loop PID and fuzzy logic controllers for right edge following and obstacle avoidance using laser sesnor. 
ROS along with python was used for controller development. 
Individual classes was developed and they are tested in Gazebo simulator.
## Fuzzy Right Edge Following controller
### Input Membership function
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_MF.PNG?raw=true)
### Rule Base
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_RuleBase.PNG?raw=true)
### Output Membership Function
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_OMF.PNG?raw=true)
### Control surface 
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/CS_REF_Str.PNG?raw=true)
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/REF_FS_CS.PNG?raw=true)

## Fuzzy Obstacle avoidance function
### Input Membership function 
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_IMF.PNG?raw=true)
### Rule Base
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_Rule_base.PNG?raw=true)
### Output Membership Function
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_OMF.PNG?raw=true)
### Control Surface
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_FS_CS.PNG?raw=true)
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/OBS_AV_ST_CS.PNG?raw=true)

## Fuzzy Coordination
Individual behaviours was merged using fuzzy coordination.
### Bahaviour selection Logic
![Classification Report](https://github.com/naveenkumarch/ROS_Feedback_controllers/blob/main/Pics/Fuzzy_Coordination.PNG?raw=true)
