# Glossary

- **eTaSL**: "expressiongraph-based Task Specification Language". It is the core software library in which robotic tasks are specified. It is based on LUA programming language and serves to declare expressions and constraints that are relevant to the task at hand. It facilitates the specification of *reactive robot behaviors*. 

- **eTaSL node**: The core library of eTaSL is independent of ROS1, ROS2, Orocos, and other robotic middleware. The eTaSL node is a reconfigurable ROS2 node that serves as an interface between eTaSL and other ROS2 nodes. One single node is able to be reconfigured online to execute different tasks in order to fulfill a robotic application. 

- **Reactive Robot Behavior**: series of actions (at the continuous level) taken by the robot which tend to follow a nominal plan, but having the ability to reactively deviate from that plan (e.g. when interacting with the human or with the environment). These behaviors are defined through task specifications and executed by closed-loop controllers.

- **Task specification**: set of constraints that determine how the states (e.g., joint positions, Cartesian twist/wrench, etc) of the robot will evolve at the continuous level, derived from how the robot should nominally move and how the robot should interact with the human, the environment, and the workpieces. In Crospi, each task specification has a series of parameters that are well-defined, bounded and documented through JSON Schemas.

- **Task**: A task is an instance (with filled-in parameters) of a task specification. Task specification -> generic to multiple contexts. Task -> specific for a certain context.

- **State coordinator/orchestrator**: refers to a software component responsible for coordinating/orchestrating the execution of different states related to a subsystem, e.g. Finite State Machines (FSMs), Behavior tree, Petri nets, etc.

- **Skill specification**: A coordinated/orchestrated set of **Tasks** that act as a building block that can be reused in different applications. For example, a Skill of "pouring water" can be composed of the following tasks 1) grasping a water jar, 2) pouring the water until the desired level, 3) placing the jar back in the table. This "pouring water" can could later on reused in different applications (e.g. for a household robot assistant or for a checmical plant). 

- **Skill**: A skill is an instance (with filled-in parameters) of a skill specification. Skill specification -> generic to multiple contexts. Skill -> specific for a certain context.

- **Twist**: Screw vector containing angular and linear velocity vectors

- **Wrench**: Screw vector containing force and moment vectors