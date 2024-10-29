# Glossary

- **eTaSL**: "expressiongraph-based Task Specification Language". It is the core software library in which robotic tasks are specified. It is based on LUA programming language and serves to declare expressions and constraints that are relevant to the task at hand. It facilitates the specification of *reactive robot behaviors*. 

- **eTaSL node**: The core library of eTaSL is independent of ROS1, ROS2, Orocos, and other robotic middleware. The eTaSL node is a reconfigurable ROS2 node that serves as an interface between eTaSL and other ROS2 nodes. One single node is able to be reconfigured online to execute different tasks in order to fulfill a robotic application. 

- **Reactive Robot Behavior**: series of actions (at the continuous level)
taken by the robot which tend to follow a nominal plan, but having
the ability to reactively deviate from that plan (e.g. when interacting
with the human or with the environment). These behaviors are defined
through task specifications and executed by closed-loop controllers.

- **State coordinator**: refers to a software component responsible for
coordinating the execution of different states related to a subsystem,
e.g. Finite State Machines (FSMs), Petri nets, etc.

- **Task**: refers to an abstraction of an action that occurs in
a robotic application and that can be fulfilled by the robot through the execution of a task specification.

- **Task specification**: set of constraints that determine how the states (e.g., joint positions, Cartesian twist/wrench, etc) of the robot will evolve at the continuous level, derived from how the robot should nominally move and how the robot should interact with the human, the environment, and the workpieces.

- **Twist**: Screw vector containing angular and linear velocity vectors

- **Wrench**: Screw vector containing force and moment vectors