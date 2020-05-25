# Projeto1-RobComp
Participants: [CEDipEngineering](https://github.com/CEDipEngineering), [lucaskf1996](https://github.com/lucaskf1996) and [jpgianfaldoni](https://github.com/jpgianfaldoni).

In this project our goal was to, give a list of goals, in the format [Color, ID, Object], where color and ID refer to descriptors of the creepers we would capture with the robot's claw, and Object was a class from the things mobileNet could detect.
We had to create a python ROS script, to allow the robot to make decisions regarding how to best behave in order to achieve its goals.
The main insturment that allowed us to produce this code in an orderly fashion, was the AI.py class, in which we made all of the logic and computation required to steer the robot based on all the gathered inputs. On the other python file (proj.py) we made use of a state machine, in which several boolean values kept track of the robot's current state, which was later adapted to be drawn on screen using the "drawStates" method of the ai class.

There were 3 goals that needed to be accomplished:

- goal1: ["blue", 11, "cat"]
- goal2: ["green", 21, "dog"]
- goal3: ["magenta", 12, "bicycle"]

Below are the videos for each goal in order:

[![Goal 1 Video](https://res.cloudinary.com/marcomontalbano/image/upload/v1590425564/video_to_markdown/images/youtube--RmPOpzUQGko-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/RmPOpzUQGko "Goal 1 Video")

[![Goal 2 Video](https://res.cloudinary.com/marcomontalbano/image/upload/v1590429784/video_to_markdown/images/youtube--2k60uD3OdDY-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/2k60uD3OdDY "Goal 2 Video")

[![Goal 3 Video](https://res.cloudinary.com/marcomontalbano/image/upload/v1590429755/video_to_markdown/images/youtube--uhBZQ8o75N4-c05b58ac6eb4c4700831b2b3070cd403.jpg)](https://youtu.be/uhBZQ8o75N4 "Goal 3 Video")

