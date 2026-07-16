# VRX Simulation Environment - Structured Learning Path

## Overview

VRX (Virtual RobotX) is a versatile simulation environment for designing, developing, and evaluating Uncrewed Surface Vessel (USV) autonomy. As of Release 3.0, it runs on **Gazebo Sim Harmonic** and **ROS 2 Jazzy** (default configuration).

---

## Prerequisites

### Required Knowledge
- Basic Linux/Ubuntu command line proficiency
- Fundamental understanding of robotics concepts
- Basic programming skills (Python and/or C++)
- Familiarity with terminal and file navigation

### Recommended Background
- Basic ROS 2 concepts (topics, services, launch files)
- Understanding of 3D coordinate systems
- Basic control theory concepts

### System Requirements
- Ubuntu 22.04 or 24.04 (for native installation)
- Minimum 8GB RAM (16GB recommended)
- Dedicated GPU with OpenGL support (NVIDIA recommended)
- ~10GB free disk space
- Docker (if choosing container-based installation)

**Reference**: https://github.com/osrf/vrx/wiki/system_requirements

---

## Recommended Learning Path

### PHASE 1: Foundation Setup (Estimated: 2-4 hours)

#### Stage 1.1: Getting Started with VRX
**Objective**: Install and run VRX for the first time

**Tutorials to Complete**:
1. **Choose Installation Method** - https://github.com/osrf/vrx/wiki/installation_method_tutorial
   - Option A: Host Machine Installation (default, recommended for beginners)
   - Option B: Docker Container Installation (for isolated environments)

2. **Prepare Your System** - https://github.com/osrf/vrx/wiki/preparing_system_tutorial
   - Install required dependencies
   - Configure system settings

3. **Install VRX** - https://github.com/osrf/vrx/wiki/installation_tutorial
   ```bash
   mkdir -p ~/vrx_ws/src
   cd ~/vrx_ws/src
   git clone https://github.com/osrf/vrx.git
   source /opt/ros/jazzy/setup.bash
   cd ~/vrx_ws
   colcon build --merge-install
   . install/setup.bash
   ```

4. **Run VRX** - https://github.com/osrf/vrx/wiki/running_vrx_tutorial
   ```bash
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
   ```

**Skills Developed**:
- ROS 2 workspace setup and building
- Environment sourcing
- Basic Gazebo simulation launch

**Key Concepts**:
- colcon build system
- ROS 2 workspace architecture
- Environment setup scripts

---

### PHASE 2: Environment Familiarization (Estimated: 3-5 hours)

#### Stage 2.1: Getting Around the VRX Environment
**Objective**: Interact with the simulation and understand basic controls

**Tutorials to Complete**:
1. **Driving the WAM-V** - https://github.com/osrf/vrx/wiki/getting_around_tutorial
   - Keyboard teleoperation
   - Basic vehicle movement

2. **Thruster Articulation** - https://github.com/osrf/vrx/wiki/thruster_articulation_tutorial
   - Understanding thruster mechanics
   - Controlling individual thrusters

3. **Adding Course Elements** - https://github.com/osrf/vrx/wiki/Adding-course-elements_tutorial
   - Customizing the environment
   - Adding obstacles and waypoints

4. **Using the Acoustic Pinger** - https://github.com/osrf/vrx/wiki/Acoustic-pinger_tutorial
   - Working with underwater sensors
   - Acoustic signal processing basics

5. **Visualizing with RViz** - https://github.com/osrf/vrx/wiki/rviz_tutorial
   ```bash
   # Terminal 1: Launch simulation
   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

   # Terminal 2: Launch RViz
   ros2 launch vrx_gazebo rviz.launch.py
   ```
   - Visualizing sensor data
   - Camera feeds and point clouds
   - Customizing RViz displays

**Skills Developed**:
- Teleoperation of USV
- Sensor visualization
- Environment manipulation
- Basic ROS 2 topic inspection

**Key Concepts**:
- WAM-V (Wave Adaptive Modular Vessel) platform
- Thruster configurations
- Sensor integration (cameras, sonar, lidar)
- Coordinate frames (REP 103/105)

---

### PHASE 3: Vehicle Customization (Estimated: 4-6 hours)

#### Stage 3.1: Customizing the WAM-V (Beginner)
**Objective**: Learn to configure vehicle components and thrusters

**Tutorials to Complete**:
1. **Using the Default WAM-V Configuration** - https://github.com/osrf/vrx/wiki/default_wamv_tutorial
   - Understanding default sensor suite
   - Default thruster configuration

2. **Creating an "Empty" WAM-V** - https://github.com/osrf/vrx/wiki/empty_wamv_tutorial
   - Building a vehicle from scratch
   - Basic component addition

3. **generate_wamv.launch.py** - https://github.com/osrf/vrx/wiki/generate_wamv_tutorial
   - Using the WAM-V generator
   - Configuration file structure

4. **Customizing the Thruster Configuration** - https://github.com/osrf/vrx/wiki/custom_thrusters_tutorial
   - Thruster placement
   - Thruster specifications
   - Performance tuning

5. **Customizing WAM-V Components** - https://github.com/osrf/vrx/wiki/custom_components_tutorial
   - Adding sensors
   - Mounting configurations
   - Component parameters

**Skills Developed**:
- XML/YAML configuration editing
- Sensor placement and configuration
- Thruster system design
- Vehicle architecture understanding

**Key Concepts**:
- SDF (Simulation Description Format)
- Component-based vehicle design
- Thruster vectoring
- Sensor mounting frames

---

### PHASE 4: Advanced Configuration (Estimated: 4-6 hours)

#### Stage 4.1: Customizing the WAM-V (Intermediate)
**Objective**: Specify custom dynamics and propulsion characteristics

**Topics Covered**:
- WAM-V hydrodynamic parameters
- Propulsion system modeling
- Dynamic behavior customization

**Reference**: https://github.com/osrf/vrx/wiki/hydrodynamic_params_tutorial

**Skills Developed**:
- Hydrodynamic modeling concepts
- Propulsion system tuning
- Performance optimization

#### Stage 4.2: Customizing Environmental Factors
**Objective**: Configure realistic maritime environments

**Tutorials to Complete**:
1. **Adjusting Wind Parameters** - https://github.com/osrf/vrx/wiki/wind_params_tutorial
   - Wind speed and direction
   - Gust effects

2. **Adjusting Wave Parameters** - https://github.com/osrf/vrx/wiki/wave_params_tutorial
   - Wave height and frequency
   - Sea state configuration

3. **Adjusting Fog Parameters** - https://github.com/osrf/vrx/wiki/fog_params_tutorial
   - Visibility conditions
   - Atmospheric effects

4. **Adjusting Ambient Light** - https://github.com/osrf/vrx/wiki/ambient_params_tutorial
   - Time of day simulation
   - Lighting conditions

**Skills Developed**:
- Environmental parameter tuning
- Realistic scenario creation
- Weather condition simulation

**Key Concepts**:
- Maritime environment simulation
- Physics-based rendering
- Sensor degradation modeling

---

### PHASE 5: Competition Preparation (Estimated: 8-12 hours)

#### Stage 5.1: Understanding Competition Tasks
**Objective**: Learn VRX competition structure and tasks

**Reference Documents**:
- VRX 2023 Task Descriptions: https://github.com/osrf/vrx/wiki/vrx_2023-task_tutorials
- VRX 2023 Technical Guide: https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview

**Competition Tasks to Study**:
1. **Stationkeeping** - https://github.com/osrf/vrx/wiki/vrx_2023-stationkeeping_task
   - Maintain position in changing conditions
   - GPS waypoint holding

2. **Wayfinding** - https://github.com/osrf/vrx/wiki/vrx_2023-wayfinding_task
   - Navigate through waypoints
   - Path planning basics

3. **Perception** - https://github.com/osrf/vrx/wiki/vrx_2023-perception_task
   - Object detection and classification
   - Computer vision for maritime objects

4. **Acoustic Perception** - https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_perception_task
   - Underwater object detection
   - Sonar signal processing

5. **Wildlife Encounter and Avoid** - https://github.com/osrf/vrx/wiki/vrx_2023-wildlife_task
   - Dynamic obstacle avoidance
   - COLREGS compliance

6. **Follow the Path** - https://github.com/osrf/vrx/wiki/vrx_2023-follow_the_path_task
   - Path following algorithms
   - Trajectory tracking

7. **Acoustic Tracking** - https://github.com/osrf/vrx/wiki/vrx_2023-acoustic_tracking_task
   - Moving target tracking
   - Signal localization

8. **Scan, Dock and Deliver** - https://github.com/osrf/vrx/wiki/vrx_2023-scan_dock_deliver_task
   - Complex multi-step operations
   - Precision maneuvering

**Skills Developed**:
- Autonomous navigation algorithms
- Computer vision for maritime applications
- Path planning and obstacle avoidance
- Multi-sensor fusion

**Key Concepts**:
- COLREGS (Collision Regulations)
- Autonomous decision making
- Real-time control systems
- Mission planning

---

### PHASE 6: Docker Packaging for Competition (Estimated: 3-5 hours)

#### Stage 6.1: Docker for VRX Competitors
**Objective**: Package autonomous system for competition submission

**Tutorials to Complete**:
1. **Docker Orientation** - https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_orientation
   - Docker basics for VRX
   - Image vs. container concepts

2. **Creating a Competitor Image** - https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_setup
   - Building custom Docker images
   - Including your autonomous stack

3. **Interactive Process** - https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_interactive
   - Testing in Docker environment
   - Debugging containers

4. **Troubleshooting** - https://github.com/osrf/vrx/wiki/tutorials-vrx_docker_troubleshooting
   - Common issues and solutions
   - Log analysis

**Skills Developed**:
- Docker containerization
- Reproducible builds
- Deployment packaging
- Debugging in containerized environments

**Key Concepts**:
- Docker images and containers
- Dependency management
- System isolation
- Submission workflow

---

### PHASE 7: RoboBoat Integration (Optional) (Estimated: 2-4 hours)

#### Stage 7.1: RoboBoat Support
**Objective**: Use VRX for RoboBoat competition preparation

**Tutorials**:
- Running RoboBoat world
- Teleoperation for RoboBoat
- Vehicle customization for RoboBoat tasks

**Reference**: https://github.com/osrf/vrx/wiki/tutorials_roboboat

---

## Learning Milestones Checklist

### Beginner Level (Phases 1-2) ✓
- [ ] VRX successfully installed and running
- [ ] Can teleoperate WAM-V using keyboard
- [ ] Can visualize sensors in RViz
- [ ] Understand basic ROS 2 commands
- [ ] Can launch and interact with simulation

### Intermediate Level (Phases 3-4) ✓
- [ ] Can customize WAM-V thruster configuration
- [ ] Can add and configure sensors
- [ ] Can modify environmental parameters
- [ ] Understand SDF configuration files
- [ ] Can create custom vehicle configurations

### Advanced Level (Phases 5-6) ✓
- [ ] Understand all competition tasks
- [ ] Can implement basic autonomous behaviors
- [ ] Can package system in Docker
- [ ] Ready to develop competition solutions
- [ ] Can debug and troubleshoot issues

---

## Key Resources

### Essential Documentation
- **System Requirements**: https://github.com/osrf/vrx/wiki/system_requirements
- **Platform Overview**: https://github.com/osrf/vrx/wiki/platform_overview
- **Frame Conventions**: https://github.com/osrf/vrx/wiki/frame_conventions
- **Troubleshooting**: https://github.com/osrf/vrx/wiki/Troubleshooting

### Competition Resources
- **VRX 2023 Overview**: https://github.com/osrf/vrx/wiki/vrx_2023-participation_overview
- **Submission Process**: https://github.com/osrf/vrx/wiki/vrx_2023-submission_process
- **Validation Testing**: https://github.com/osrf/vrx/wiki/vrx_2023-validation
- **WAMV Compliance**: https://github.com/osrf/vrx/wiki/vrx_2023-wamv_compliance

### External Resources
- **ROS 2 Documentation**: https://docs.ros.org/en/jazzy/
- **Gazebo Sim Documentation**: https://gazebosim.org/docs/harmonic
- **VRX GitHub Repository**: https://github.com/osrf/vrx

---

## Time Investment Summary

| Phase | Description | Estimated Time |
|-------|-------------|----------------|
| Phase 1 | Foundation Setup | 2-4 hours |
| Phase 2 | Environment Familiarization | 3-5 hours |
| Phase 3 | Vehicle Customization (Beginner) | 4-6 hours |
| Phase 4 | Advanced Configuration | 4-6 hours |
| Phase 5 | Competition Preparation | 8-12 hours |
| Phase 6 | Docker Packaging | 3-5 hours |
| Phase 7 | RoboBoat Integration (Optional) | 2-4 hours |
| **Total** | Complete Learning Path | **26-42 hours** |

---

## Tips for Success

1. **Follow tutorials in order** - Each tutorial builds on previous knowledge
2. **Practice regularly** - Consistent practice is better than long sessions
3. **Experiment freely** - Modify parameters and observe effects
4. **Read error messages carefully** - Most issues are documented in Troubleshooting
5. **Join the community** - Engage with VRX community for support
6. **Start simple** - Master basics before attempting complex tasks
7. **Document your work** - Keep notes on configurations and solutions

---

## Next Steps After Completion

After completing this learning path, you will be ready to:
1. Develop autonomous navigation algorithms
2. Implement computer vision systems for maritime applications
3. Participate in VRX competitions
4. Contribute to VRX development
5. Apply skills to real-world USV projects
6. Pursue research in maritime robotics

---

*Learning path created from VRX Wiki documentation: https://github.com/osrf/vrx/wiki*
*Last updated: Based on VRX Release 3.0 (Gazebo Sim Harmonic + ROS 2 Jazzy)*
