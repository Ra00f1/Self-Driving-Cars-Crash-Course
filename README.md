# Self-Driving Cars Crash Course(6 Days)

## Introduction 
### What Are Self-Driving Cars?
Self-driving cars, also called autonomous vehicles (AVs), are vehicles capable of sensing their environment and moving safely with little or no human input. These cars use a combination of hardware (like cameras, radars, LiDAR, GPS, and inertial measurement units) and software (such as computer vision, deep learning, sensor fusion, and planning algorithms) to perceive their surroundings, make decisions, and control the vehicle.

### What We Need to Learn
To understand and contribute to self-driving car systems, we need to cover the following key areas:
* **Perception**: How the vehicle senses and understands the environment using cameras, LiDAR, radar, and other sensors.
* **Computer Vision and Deep Learning**: Techniques for detecting objects, segmenting scenes, recognizing lanes, and interpreting traffic signs.
* **Sensor Fusion**: Combining data from multiple sensors to build a more accurate model of the world.
* **Localization**: Determining the vehicle’s position within a map using visual odometry, GPS, or SLAM (Simultaneous Localization and Mapping).
* **Path Planning**: Deciding how the vehicle should move, avoid obstacles, and follow traffic rules.
* **Control Systems**: Executing the planned path by controlling the steering, throttle, and brakes smoothly.

### Technologies and Methods Used
Self-driving car development involves a wide range of technologies, including:
* Cameras and Image Processing for detecting lanes, traffic signs, vehicles, pedestrians, and more.
* LiDAR and Radar for 3D environment mapping and distance measurements.
* Deep Learning Models like CNNs for object detection and semantic segmentation.
* Sensor Fusion Algorithms like Kalman Filters to combine multiple sensor inputs.
* Localization Techniques such as GPS data correction and visual SLAM.
* Simulation Environments (like CARLA and Duckietown) for testing and validation.
* Real-Time Systems and ROS (Robot Operating System) to coordinate software modules and ensure reliability.

Source: https://www.coursera.org/learn/intro-self-driving-cars?specialization=self-driving-cars

### What we'll learn
* Understand the key methods for parameter and state estimation
* Develop and use models for typical vehicle localization sensors
* Apply Kalman filters to a vehicle state estimation problem
* Register point clouds from LIDAR to 3D Maps

# Day 1

- Driving Task
  - Preceiving teh environment
  - Planning a path
  - Controlling the vehicle
- Operational Design Domain(ODD)
### Driving Task
* Lateral Control
  * Steering
  * going from left to right
* Longitudinal Control
  * Breaking
  * Accelerating
* Object and Event Detection and Responce(OEDR)
* Planning
  * Long term
  * Short term
#### Autonomous Car Levels
* Level 0: no automation
* Level 1: Either Lateral or Longitudial
  * Adaptive Cruise Control
  * Lane keeping Assistance
* Level 2: Both Lateral and Longitudial
  * Driver assistance needed
* Level 3: Level 2 + OEDR
  * when fails driver needs to take control
* Level 4: Level 3 + Fallback
  * can handle emergencies if driver doesn't take control
* Level 5: Level 4 + unlimited ODD

## Perception
**Perception**: can it see something and understand it.

For example: Is it a car? Where is it? How is it moving?

![image](https://github.com/user-attachments/assets/1bb3f756-4101-4f9d-afe3-68dca09264f5)

### Goals of Perception
* Static Objects
  * on-road: Road and lane markings, Construction signs
  * off-road: Curbs, Traffic lights, Road signs
* On-road Dynamic Objects
  * Vehicles
    * 4 wheelers
    * 2 wheelers(harder to understand how they will move as hey have more freedom than 4 wheelers)
  * Pedestraisn
    * more erratic
  * Ego Localization
    * Position, Velocity, Acceleration, Orientaton, Angular motion
    * GPS, IMU, odometry sensors are used to get the data needed
   
## Planning
<img src="https://github.com/user-attachments/assets/098766c3-65a2-40cb-9d1e-26b62c960ac7" width="500"/>

* Long Term
  * The path to take from pont A to B
* Short term
  * When to change lane
  * When to pass
* Immediate
  * Accelerate or brake

Planning Methods: 

* **Reactive Planning(Rule Based Planning)**
  * If there is a pedestrian on the road, stop.
  * If speed limit changes, adjust speed to match it.

* **Predictive Planning**
  * That car has been stopped for the last 10 seconds. It is going to be stopped for the next few seconds.
  * Pedestrian is jaywalking. She will enter our lane by the time we reach her so I need to slow down.

## Hardware 

### **Sensors**
<img src="https://github.com/user-attachments/assets/ce013be8-84e3-4313-a3c7-b59ece3713b0" width="800"/>


* exteroceptive: extero = surroundings
* proprioceptive: proprio = internal
### Exteroceptive Sensors
#### Cameras 
* Metrics
  * Resolution
  * Field of view (FOV)
  * Dynamic Range
* Stereo cameras: image produced by focusing two cameras into the same angle/frame
#### Lidar
* Creates a 3D image by shooting light to the environment and absorb the reflected light.
* Metrics
  * Number of beams(common size is 64)
  * Points per second(how many lpoints can absorb per second)
  * Rotation rate(the higher the faster the upadte of the 3D image is)
  * FOV
* Solid state Lidars don't have rotation
#### Radar
* used for object detection
* not affected by weather
* Metrics
  * Range
  * FOV
  * position & speed accuracy
* types:
  * Wide FOV, short range
  * Narrow FOV, long range
#### Sonar
* used for parking and clsoe range

### Proprioceptive Sensors
 #### GNSS/IMPU/GPS
 * Global Navigation Satellite Systems(GNSS)
 * Inertial measurement units(IMU)
 * measures of ego vehicle states
 * GNSS:
   * Position
   * Velocity
  
 * IMU:
   * Angular rotation
   * Acceleration
   * Heading(IMU + GPS)
#### Wheel Odometry
* Tracks wheel rotation
* Estimates the speed and Orientation of the vehicle

## Computing Hardware 
* Most used hardware:
  * GPU
  * FPGA
  * ASIC

## Sensor Configuration for Autonomous Vehicles
### Common Sensors
* Camera – Appearance input
* Stereo Camera – Depth perception
* LIDAR – All-weather 3D sensing
* Radar – Object detection
* Ultrasonic – Short-range 3D detection (e.g., parking)
* GNSS/IMU/Wheel Odometry – Ego state estimation

### Deceleration and Stopping Distance
#### Braking Dynamics
* Aggressive deceleration: 5 m/s² (emergency)

* Normal deceleration: 2 m/s² (comfortable)

* Formula: d= v^2 / 2a
  * d = stopping distance (how far the vehicle travels before coming to a complete stop)
  * v = initial velocity of the vehicle (in meters per second, m/s)
  * a = deceleration (rate of slowing down, in meters per second squared, m/s²)
 
#### Highway Driving – Sensor Coverage
**Key Maneuvers**
* Emergency braking
* Maintaining speed
* Lane changing

**Sensor Requirements**
* Forward sensing:
  * Emergency stop: 110–200 m
  * Following vehicles: 65–100 m
* Lateral sensing:
  * Adjacent lanes: ~3.7 m each
  * Field of view: 160°–180°
  * Range: 40–60 m
* Rear and side awareness:
  * Required for lane changes and merges
  * 360° situational awareness for safety

#### Urban Driving – Sensor Coverage
**Additional Maneuvers**
* Overtake parked vehicle
* Intersection navigation (left/right turns)
* Roundabout navigation

**Sensor Requirements**
* Overtaking:
  * Parked car: short-range wide FoV
  * Oncoming traffic: narrow long-range
* Intersections:
  * Omni-directional detection (vehicles + pedestrians)
* Roundabouts:
  * Short-range, wide-angle longitudinal and lateral sensing

#### General Sensor Strategy
**Design Principles**
* Long-range, narrow FoV:
  * Detect distant longitudinal hazards
* Short/medium-range, wide FoV:
  * 360° coverage within ~50 m
* Ultrasonic/Sonar:
  * Useful for very close range (e.g., parking)

#### Design Considerations
* Sensor setup should be based on:
* Required maneuvers (ODD-specific)
* Forward and lateral coverage needs
* Redundancy (in case of sensor failure)
* Budget and hardware limitations

## Software
Softwares architecture:
* Environment Perception
* Environment Mapping
* Motion Planning
* Controller
* System Supervisor

<img src="https://github.com/user-attachments/assets/b5029f8b-0ed7-4cbc-a632-b501f4d985ae" width="650"/>

Image source: https://fisher.wharton.upenn.edu/wp-content/uploads/2020/09/Thesis_Nova-Qiaochu-Guo.pdf

Localization/Environment Mapping:
* Occupancy Grid Map

<img src="https://github.com/user-attachments/assets/4bcf4c98-a932-4dbe-b32b-c679fceff802" width="400"/>

* Localization Map

<img src="https://github.com/user-attachments/assets/da0bc474-ec0c-472d-9ec3-db96cf76c897" width="400"/>

* Detailed Road Map

<img src="https://github.com/user-attachments/assets/4d9a9000-ceed-42ce-b37a-45c5d55e4bdf" width="400"/>

Planning:

<img src="https://github.com/user-attachments/assets/db984273-f32a-4a0e-a5d0-446c54ad733e" width="600"/>


# Day 2
## Fully Convolutional Network
<img src="https://github.com/user-attachments/assets/a15d2b5c-167c-433c-9cb8-3c06e5b54033" width="650"/>

Image source: Chen, Peidong & Su, Xiuqin & Liu, Muyuan & Zhu, Wenhua. (2020). Lensless Computational Imaging Technology Using Deep Convolutional Network. Sensors. 20. 2661. 10.3390/s20092661. 


# Glossary of Terms
**ACC: Adaptive Cruise Control**
A cruise control system for vehicles which controls longitudinal speed. ACC can maintain a desired reference speed or adjust its speed accordingly to maintain safe driving distances to other vehicles. 

**Ego**
A term to express the notion of self, which is used to refer to the vehicle being controlled autonomously, as opposed to other vehicles or objects in the scene.  It is most often used in the form ego-vehicle, meaning the self-vehicle.

**FMEA: Failure Mode and Effects Analysis**
A bottom up approach of failure analysis which examines individual causes and determines their effects on the higher level system.

**GNSS: Global Navigation Satellite System**
A generic term for all satellite systems which provide position estimation. The Global Positioning System (GPS) made by the United States is a type of GNSS. Another example is the Russian made GLONASS (Globalnaya Navigazionnaya Sputnikovaya Sistema).

**HAZOP: Hazard and Operability Study**
A variation of FMEA (Failure Mode and Effects Analysis) which uses guide words to brainstorm over sets of possible failures that can arise.

**IMU: Inertial Measurement Unit**
A sensor device consisting of an accelerometer and a gyroscope. The IMU is used to measure vehicle acceleration and angular velocity, and its data can be fused with other sensors for state estimation.

**LIDAR: Light Detection and Ranging**
A type of sensor which detects range by transmitting light and measuring return time and shifts of the reflected signal.

**LTI: Linear Time Invariant **
A linear system whose dynamics do not change with time. For example, a car using the unicycle model is a LTI system. If the model includes the tires degrading over time (and changing the vehicle dynamics), then the system would no longer be LTI.

**LQR: Linear Quadratic Regulation**
A method of control utilizing full state feedback. The method seeks to optimize a quadratic cost function dependent on the state and control input.

**MPC: Model Predictive Control**
A method of control whose control input optimizes a user defined cost function over a finite time horizon. A common form of MPC is finite horizon LQR (linear quadratic regulation).

**NHTSA: National Highway Traffic Safety Administration**
An agency of the Executive Branch of the U.S. government who has developed a 12-part framework to structure safety assessment for autonomous driving.  The framework can be found here. 
https://www.nhtsa.gov/sites/nhtsa.dot.gov/files/documents/13069a-ads2.0_090617_v9a_tag.pdf

**ODD: Operational Design Domain**
The set of conditions under which a given system is designed to function. For example, a self driving car can have a control system designed for driving in urban environments, and another for driving on the highway.

**OEDR: Object and Event Detection and Response**
The ability to detect objects and events that immediately affect the driving task, and to react to them appropriately. 

**PID: Proportional Integral Derivative Control**
A common method of control defined by 3 gains.
1) A proportional gain which scales the control output based on the amount of the error
2) An integral gain which scales the control output based on the amount of accumulated error
3) A derivative gain which scales the control output based on the error rate of change

**RADAR: Radio Detection And Ranging**
A type of sensor which detects range and movement by transmitting radio waves and measuring return time and shifts of the reflected signal.

**SONAR: Sound Navigation And Ranging**
A type of sensor which detects range and movement by transmitting sound waves and measuring return time and shifts of the reflected signal. 
