---
title: Engineering Portfolio
subtitle: Click on an image to get started.
---
| [![NRSL](/assets/png/nrsl_link.png)](#NRSL){: .mx-auto.d-block :} | [![THON](/assets/png/thon_bot_link2.png)](#THON){: .mx-auto.d-block :} | [![TonyPi](/assets/png/tonypi_link.png)](#tonypi){: .mx-auto.d-block :} 
| [![Drumstick](/assets/png/drumstick_link.png)](#DRUM){: .mx-auto.d-block :} | [![AstaZero](/assets/png/astazero_link.png)](#drone){: .mx-auto.d-block :} | [![ACRP](/assets/png/acrp_link.png)](#ACRP){: .mx-auto.d-block :}


## Networked Robotic Systems Lab at Penn State University {#NRSL}

During my undergraduate experience at Penn State University, I became the lead undergraduate researcher for the Networked Robotic Systems Lab (NRSL). The project that I spent most of my time with used A* and path planning algorithms in MATLAB to navigate an unfamiliar area. This robot, which I helped prototype for lab demonstration, would connect to a Vicon motion capture system to record its position and determine possible trajectories.

![Jake Bot Diagram](/assets/png/JakeBot.png){: .mx-auto.d-block :}

The lab became unused during the pandemic, so my day-to-day duties also included developing a fleet of mobile robots that can be used in a variety of situations. These robots could be programmed via Arduino to run control or path planning algorithms in a graduate student lab setting or be upgraded with Jetson boards for research use.

![Kraus Researching](/assets//undergrad_research.png){: .mx-auto.d-block :}


## THON Bot: 5 Foot Tall Dancing Humanoid Robot {#THON}
As a way to build advanced experiences for the Robotics Club, I created the THON Bot project. THON Bot is a 5 foot humanoid dancing robot that will interact with attendees for 48 hours continuously at THON 2025, which is a dance marathon that raises funds for pediatric cnacer. This is one of the most complicated and rewarding robotics projects that I have experienced at Penn State University; there are electronics and controls problems that I would not have encountered as an undergraduate in Mechanical Engineering.

As the founder of the project, I divided the project into 3 teams: Hardware, Electronics, and Simulation. On the Hardware team, I oversaw a team of interdisciplinary undergraduate engineers that designed, 3D printed, and assembled plastic and metal components of THON Bot.   

![THON Bot Diagram](/assets/png/thon_bot_diagram.png){: .mx-auto.d-block :}

In the future, THON Bot has a variety of uses outside of the THON dance marathon. The project has functions and hardware that would benefit Human-Robot Interaction research or answer ethical questions about the nature of robots in society.


## Tony Pi Humanoid Robot Projects {#tonypi}
During my senior year, I had the opportunity to enroll in the recently developed humanoid robotics class at Penn State. Because of the freeform nature of the course, I learned a variety of software related to devleoping code for a TonyPi humanoid robot. Code was written in Python and sent to an onboard Raspberry Pi to make the robot perform a variety of tasks such as dancing, performing exercises, and stacking wooden blocks. As part of the course, I performed a stabililty analysis based on accelerometer and presssure data to determine which movements were more stable than others and what factors play into stable bipedal motion.


[Link to Dance](/assets/mov/TonyPi_Dance.mp4)

The class also taught OpenCV fundamentals using the robot's onboard camera system. For one of my projects, I tuned a PID controller to control the servo motors on the camera to track the centermost face.

[Link to Track](/assets/mov/TonyPi_Track.mp4)

## Drumstick: Testing Quadruped Leg Principles {#DRUM}
To prepare for graduate school, I decided to construct a quadruped leg in my free time over the summer using off-the-shelf components and 3D printed parts. The design of the leg is loosely based on projects from the Open Dynamics Robot Initiative (https://open-dynamic-robot-initiative.github.io), but I replaced the pancake motors with drone motors I already had with a 100:1 gearbox for increased torque. Also, the knee linear actuator common across different quadruped designs has been replaced with a motor directly attached to the joint for ease of use.

![Drumstick Diagram](/assets/png/drumstick_diagram.png){: .mx-auto.d-block :}

One of the areas I am particularly interested in studying further is how to modify electromechnaical designs to increase the capabilities of mobile robots. Most of the parts in the quadruped leg are replaceable and can be modified with tools common to makerspaces; this allows me to implement changes in both the control systems and the hardware design of the robot to deepen my understanding of specific concepts. 

## Autonomous Surveillance of Restricted Area Using Drones {#drone}
AstaZero AB has an autonomous vehicle testing and research facility in a heavily wooded area in Sweden. This facility is surrounded by an approximately 10 kilometer fence to prevent moose, deer, and other animals from interrupting outdoor experiments. The team had the opportunity to automate this process using existing drone hardware at AstaZero.

A line detection software was created using Python and the openCV programming library. The code isolates the fence in the captured drone video and flags any gaps in where the fence appears.

The drone flight program is built using the DSS (Drone Security System) library and Python code. Perimeter fence GPS coordinates are recorded into the system and the drone flies to a certain altitude to detect the fence, reaches all GPS locations, and lands safely.

Overall, the project was chosen for Best Project Award out of nearly one hundred senior student teams at Penn State University. The project was tested at AstaZero's track in Sweden successfully and executed the mission with assistance from the Chalmers University teammates. 

[Link to Poster](/assets/AstaZeroAB%20Team%201%20Poster.pdf)

[Link to Video](https://www.youtube.com/watch?v=jhPUywB5TlE)



## Airport Cooperative Research Project (ACRP) Design Challenge {#ACRP}
The Airport Cooperative Research Program is a national competition for university students and was a component of one of my classes in the Engineering Leadership Development minor program. The competition tasked students with improving one aspect of an airport terminal and operations in a thoughtful and well-researched way; everything was to be submitted in a final report for industry experts and judges.  

Our group decided to investigate an aspect of an airport that we could directly study and improve: the terminal experience for elderly passengers. Through family interviews, field research at a local airport, and copious research on the elderly population in America, we decided to create a Bluetooth-based device that directs passengers to bathrooms, food courts, gates, and other services. Because of our efforts, we were awarded 2nd Place nationally in the Airport Management and Planning category.

[Link to Video](https://www.youtube.com/watch?v=CzLzuzDaduI) 

[Link to Report](https://williamkraus.files.wordpress.com/2021/10/l.e.n.d.-acrp-challenge-report.pdf)


