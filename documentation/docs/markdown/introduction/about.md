# About

Crospi was born after a long list of open-source constraint-based task specification methods developed at KU Leuven university. After several years of experience, we identified several flaws and improved upon them. 

??? note "A summary of constraint-based frameworks :material-cursor-default-click:"

    In literature, there exist many approaches in which autonomous robots can be programmed in order to execute tasks. A common paradigm which many approaches follow is the so-called Sense-Plan-Act, in which the robot: 1) gathers information about the environment using sensors, 2) elaborates a plan on how to perform the task, and 3) finally executes the plan to achieve the task. In contrast, a more robust way to deal with changes in the environment, or other uncertainties related to the task, is to instead rely on reactive approaches. These approaches continuously use sensor information to react to changes and continuously adapt the defined actions based on control laws. 

    There are several approaches that offer such reactiveness during task executions. A possibility is to specify nominal actions (e.g., trajectories) in combination with descriptions on how to deviate from these nominal actions based on sensor information. The resulting behaviors due to these combinations are referred to as **reactive robot behaviors**. 

    A quite convenient way of defining such reactive robot behaviors is to model them as a set of **constraints**. For example, if a robot needs to insert a peg into a hole:  (i) the peg should maintain alignment with the hole (orientation and position constraints which are part of the nominal plan),  (ii) the robot should maintain a certain velocity in the direction of insertion (velocity constraint which is part of the nominal plan),  (iii) the robot should act compliantly to the contact between the peg and the hole since there are uncertainties associated with the location or with manufacturing tolerances of both workpieces (force constraints which are part of the deviations from the nominal plan). 

    Of course, this modeling is not unique and the selection of such constraints will influence the behavior and performance of the task execution, with pros and cons under certain situations. 

    Methods that model a task by using a set of constraints are referred to as constraint-based methods. Our group has developed several constraint-based frameworks, including the [task frame formalism](https://ieeexplore.ieee.org/abstract/document/508440),  [iTaSC](https://orocos.org/itasc.html) and [eTaSL](https://etasl.pages.gitlab.kuleuven.be/). 

    The first framework, the [task frame formalism](https://ieeexplore.ieee.org/abstract/document/508440), allows the specification of force-controlled actions to cope with geometric uncertainties that are too large to handle with only passive (i.e., mechanical) compliance. The formalism allows the specification of either force or velocity constraints for each of the axes of a reference frame (both linear and angular), independently. This reference frame will be referred to as the task frame, and hence it is selected according to the task at hand. After these actions are defined, a low-level controller is needed to execute the specified forces and velocities. 

    The second framework, [iTaSC](https://orocos.org/itasc.html), provides a constraint-based approach for specifying and controlling robot tasks by formulating them as sets of equality and inequality constraints on task-related variables such as position, velocity, and force. Rather than relying on predefined trajectories, [iTaSC](https://orocos.org/itasc.html) allows robots to adapt in real time by continuously solving these constraints using sensor feedback. This enables flexible, reactive behaviors that can handle dynamic environments and uncertainties, making [iTaSC](https://orocos.org/itasc.html) especially suitable for tasks involving interaction with humans or unstructured settings. 

    The third framework, [eTaSL](https://etasl.pages.gitlab.kuleuven.be/), which is used by Crospi, shares a similar philosophy as [iTaSC](https://orocos.org/itasc.html) but with a different implementation that offers better performance, more generality, easier usage, and less singularities. This framework also offers more flexibility and possibilities for specifying constraints related to the task, to the robot, and to the environment. 

    The most important elements used to create a task specification in [eTaSL](https://etasl.pages.gitlab.kuleuven.be/) are twofold: expressions and constraints. Firstly, expressions need to be defined, which depend on robot-related variables (i.e., joint positions), and/or variables describing features of the task (referred to as feature variables), and/or time. Secondly, some constraints over these expressions need to be declared. 

    There are two types of constraints that can be specified for each of the expressions: 

    - Equality or inequality constraints imposed on the expression itself (i.e., position level). 
    - Equality or inequality constraints imposed on the time derivative of the expression (i.e., velocity level). 

    For example, a commonly used expression is the position of the end effector of the robot (which depends on the joint positions), and different constraints can be imposed on it. One can impose, for instance, the following constraints: 

    - A position equality constraint, such that it follows a desired trajectory. 
    - A velocity equality constraint, such that it follows a desired velocity coming from an input device such as a joystick. 
    - A velocity equality constraint, such that the end effector moves proportionally to measured end-effector forces (i.e., admittance constraint), so that it remains compliant. 
    - Position inequality constraints to ensure it remains within a safety region. 

    All of the above constraints can be combined into the same task specification if desired. Since it is possible that not all of them can be fulfilled at the same time, eTaSL allows the definition of two priority levels: 

    - Constraints with the highest priority are referred to as hard constraints, as they must be fulfilled; otherwise, the task is considered unfeasible. 
    - Constraints with the lowest priority are referred to as soft constraints and always have an associated weight that indicates their relative importance. 

    For example, if there are only two soft position constraints with the same weight, the resulting behavior is that the expression (e.g., the end effector) will be controlled toward the midpoint between the two targets. It is important to mention that this weight does not influence stability—it only affects behavior when there are other conflicting constraints. 

## Understanding the Crospi Pipeline

The figure below illustrates Crospi hierarchical architecture. The architecture separates the concerns of robot application development into four blocks, each with its corresponding stakeholder. Thanks to Crospi, the required knowledge about the usage of the libraries provided to the next stakeholder (green arrows in the diagram) is minimized, since all of these come with JSON Schemas that makes it easy to use the content of the libraries. Different tools to manage and create the corresponding JSON Schemas of the different blocks are provided by Crospi. 

<figure>
  <div id="svg-container" style="width:100%;"></div>
  <figcaption>
    <h6 align="center">Click on the different blocks to jump to the corresponding tutorials.</h6>
  </figcaption>
</figure>


## Crospi vs eTaSL
[eTaSL](https://etasl.pages.gitlab.kuleuven.be/) is a core dependency of Crospi. While Crospi is a highly-configurable pipeline that focuses on interfacing (plugins, ROS2 and non-ROS sensors, robot hardware, easy user interfaces) and easy integration (pipeline for Orchestration, creation of libraries, configuration files, etc), [eTaSL](https://etasl.pages.gitlab.kuleuven.be/) is the language to define the task specifications. [eTaSL](https://etasl.pages.gitlab.kuleuven.be/) is a collection of C++/CMake/LUA libraries that are used within Crospi, but that do not depend on Crospi. In summary: all the complex real-time C++ code was already done for you with Crospi, and therefore you can focus on configuring it to use your own robot setup in your own orchestrated applications. 


## Why is this so powerful?

Crospi enables powerful and reactive robotic behaviors, while reducing the expertise required by the different stakeholders. It facilitate the work of all stakeholders and them to understand eachother's input by defining in a systematic way their roles and developments. We will explain it's power through the following case studies:


### Case Study 1

**Background:** this usecase was fully implemented with Crospi and obtained the **second place** in the [Neura Robotics Challenge 2025](https://neura-robotics.com/neura-robotics-challenge/) and a price of **35 thousand** euros.

The use case showcases mobile manipulation for a cable harnessing application. Our solution consists of the Neura's MAV platform working together with the Neura's MAiRA arm to deal with larger assembly boards and larger cable racks. Similarly to human operators, we use force-based skills where the robot actively keeps the cable under a desired tension while manipulating it. These force-based skills require low-level control (i.e. closed-loop control at 1kHz) that is handled by Crospi and configured by the user. The real-time controllers at each stage are automatically generated from higher-level eTaSL task specifications.  

The use case uses MAV’s mobility while compensating for its slower dynamics using the faster dynamics of the MAiRA robot arm. This enables us to use MAV+MAiRA simultaneously and together to execute the sensor-based smart skills necessary to perform the cable routing in a reliable way. These smart skills also tackle the variability related to cables (i.e. mechanical and geometrical properties), allowing the robot to manipulate and route the cables with ease and robustness. Aditionally, in the mobile manipulation context, where the workpieces with respect to the robot cannot be fixed, having a combination of vision (to get approximate workpiece locations) and force (to deal robustly with geometric uncertainties) becomes essential. Crospi seamlessly enables this integration. 

<iframe width="560" height="315" 
        src="https://www.youtube.com/embed/vRGSpFZ4Pgo?si=c3_73sTiIky0pddN" 
        title="YouTube video player" frameborder="0" 
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
</iframe>

### Case Study 2

**Background:** This use case obtained the **first price** at the [Kuka Innovation Award 2021](https://www.kuka.com/en-be/future-production/innovation-and-research/kuka-innovation-award/kuka-innovation-award-2021) and a price of **20 thousand** euros. This usecase was not implemented with Crospi but with a previous version of eTaSL running on an Orocos component. Nevertheless, Crospi offers all the tools to implement the use case in a faster and cleaner way.

Assume that a new robotic application needs to be developed and deployed for an SME. The batch sizes for an SME are small, so pursuing a hard automation solution is not economically feasible. Instead, an automation solution that can be deployed in a short time and at a low cost is desired.

This is exactly the challenge tackled in this use case, enabling SMEs to comission dual-arm robotic tasks in a rapid way. The fast deployment was achied through a combination of constraint-based task specifications and learning from demonstrations. A total of four tasks where presented to demonstrate the idea: o-ring assembly by learning motions and interaction forces from human demonstrations, force-based contour-following inspection of previously-uncknown geometries, coordinated force-based solenoid insertions, and beer bottle opening by learning motions and interaction forces from human demonstrations.


<iframe width="560" height="315" 
        src="https://www.youtube.com/embed/sCx1HB9-kgM?si=Fg9xeL2nWkKDEIns" 
        title="YouTube video player" frameborder="0" 
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
</iframe>

### Case Study 3

**Background:** This use case run for several years at the factory of Audi Brussels. This usecase was not implemented with Crospi but with a previous version of eTaSL running on an Orocos component, and was not implemented directly by KU Leuven but by the company *FRS Robotics*. Nevertheless, Crospi offers all the robot-related and interfacing-related software tools (not the tools developed specifically by FRS robotics for this use case) to implement the use case.



<iframe width="560" height="315" 
        src="https://www.youtube.com/embed/Ht-8UFmpsac?si=-J4C3FF2Bd9M5GqQ" 
        title="YouTube video player" frameborder="0" 
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
</iframe>

### Case Study 4

**Background:** This usecase was not implemented with Crospi but with a previous version of eTaSL running on an Orocos component, and was implemented in collaboration between KU Leuven and *FRS Robotics*. Nevertheless, Crospi offers all the robot-related and interfacing-related software tools (not the tools developed specifically by FRS robotics for this use case) to implement the use case.

Automating cheese decrusting is a challenging task since natural products such as cheese does not have a completely standard shape. Instead, there are a lot of variations related to the cheese geometry. Having these variations into account during the automation process is economically crucial, since all the wasted cheese will have a dramatical impact in the company's economy. 

By sensing the geometry of the cheese with laser sensors during the decrusting, the robot is able to reactively adapt the trajectory based on the geometry of the cheese. With this strategy a lot of cheese-waste was avoided! 


<iframe width="560" height="315" 
        src="https://www.youtube.com/embed/GAx9dE-vgBs?si=vL88qg-gLcqfz6GX&amp;start=270" 
        title="YouTube video player" frameborder="0" 
        allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen>
</iframe>



<!-- <figure>
<p align="center" width="100%">
<img src="/images/stakeholders_and_task_scheme.svg" alt="init_result" style="width:100%; display: block; margin: 0 auto">
<figcaption><h6 align="center">Fig. 1 - hierarchy diagram of used terms.</h6></figcaption>
</p>
</figure> -->

