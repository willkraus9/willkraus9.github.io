---
title: Current Projects
subtitle: Here is what I've been working on currently, with some updates from time to time. 
---

## Current Research (IROS Publication Likely): Gait Optimization for Simplified Legged Robot Model

In my current research in the REx Lab, I am working on an ongoing project with legged locomotion that simplifies model-based control strategies. For my portion of the project, I am researching how this framework changes gait requirements for stability to generate predictable and stable movements. So far, I have done prelimiary experiments using constrained trajectory optimization in a DDP formulation. 

(insert GIF of nadia moving)

So far, this needs some attention, but improvements are possible :)

Current TODOs for this project are: 
* Penalize spinal movement (perhaps through regularization, penalizing spinal movements, or by fixing parts of the URDF). The robot model comes custom from IHMC's Nadia project, so accuracy shouldn't be a concern. 
* The scope of the problem should include whole body control, not just legged locomotion. Currently, I only take into account the legs in the model, hence why the body flails around over two steps. Ideally, the robot should have the potential for loco-manipulation tasks, but this might be out of the scope of the research paper if my collaborators and I are solely focused on walking. 

Send me an email if you have any suggestions! I'm always looking for advice: will.kraus9@gmail.com

## Volunteer Project: Bimanual Manipulation Setup on Baxter robot for Local High School

One of the school districts near where I went to high school were given a Rethink Robotics Baxter robot and they want to integrate robotics design and Python programming into their curriculum. This would be an *excellent* skills-building project that I would have loved to be a part of when I was younger, but the school district does not have anyone with robotics development or ROS experience. 

Fortunately, I have been in contact with the Director of Technology for the school district and the high school robotics teacher for the past year or so and have guided them on how to integrate Baxter into their curriculum, what configurations for the computer lab they need to be in compliance with their network security, and how to get Baxter to work. 

So far, the initial results look promising! I'm looking to wrap up the project after a few more in-person visits for the hardware. We'll probably use some sort of Python-based Pinocchio wrapper to control Baxter (unless students are learning ROS, which can be challenging to newcomers). Potential projects include pick-and-place tasks, human-robot interaction studies, and designing end effectors!

(insert image from post)


