# CG2111A — Engineering Principles and Practice II Project

This group project was completed as part of the CG2111A Engineering Principles and Practice II course taught at the National University of Singapore (NUS). The goal of the project was to create a teleoperated vehicle capable of navigating a room filled with various obstacles, mapping out its surrounding environment, and identifying victims. For more informtion and details, view the full final report [here](<Reports/Final Report.pdf>).

## Section 1 | Introduction

Alex is a robotic vehicle that is tasked to perform search-and-rescue missions. In this prototype phase, Alex will be placed in a simulated environment where it is easier to navigate and detect victims. It is tasked to navigate a room filled with various man-made obstacles such as boxes, map out its surrounding environment, and identify victims. In this test, victims are either “green” – healthy but trapped, or “red” – injured and trapped. Moreover, Alex must be teleoperated so that operators are not in contact with the disaster site. The goal for the teleoperator is to locate two victims and correctly identify their statuses, map out the disaster site, and park Alex in a designated spot – all within a 6-minute window, with minimal collisions.

To this end, Alex is designed with the following specifications and functionalities. A Raspberry Pi is installed on Alex to allow for communication between the operator’s laptop and the Arduino controlling its components. The LiDAR sensor, in conjunction with Hector SLAM, allows Alex to detect obstacles and objects while simultaneously identifying its position on the map. To differentiate victims, healthy or injured, from obstacles, a colour sensor is installed on Alex. If the detected colour is not red or green, Alex will identify the object as an obstacle instead of a victim. Additionally, ultrasonic sensors are placed at the front, left and right sides on Alex to detect and avoid nearby obstacles. Alex also features a loud buzzer, used to alert the teleoperator and nearby pedestrians when a victim has been detected.

## Section 2 | Review of the State of the Art

### 2.1 | Boston Dynamics– Atlas

The Atlas is a humanoid robot designed to assist emergency management teams in handling natural and
man-made catastrophes. It can perform tasks such as flipping switches, shutting off valves, opening doors, and running power equipment. These functions are possible through the use of components such as lightweight
hydraulics and 3D-printed appendages. It also uses LiDAR and stereo vision to effectively navigate through
rough terrains.

Strengths:
- 3D printing is used to manufacture components to save weight and space, resulting in a compact robot with a high strength-to-weight ratio
- Advanced control algorithms enable the robot to plan and perform actions based on the environment it analyses

Weaknesses:
- Too large to fit in tight spaces, inefficient in spotting and saving casualties in such spaces
- Ineffective in other terrains, such as water and air, hence only useful for land
operations
- Usesalot of energy, which may not be sustainable for long operations
