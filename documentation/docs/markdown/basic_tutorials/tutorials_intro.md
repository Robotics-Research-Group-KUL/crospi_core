# How to Follow These Tutorials

There are two ways of navigating the tutorials, depending on whether you are learning alone or working in a team.


??? note "Working Solo :material-cursor-default-click:"

    If you are learning Crospi on your own, we recommend following the tutorials in sequence to gradually build a complete understanding of the workflow.

    - **Step 1 — Getting Started:** Begin with the **Installation**, **Templates**, and **How to Run** tutorials. Then explore the **Examples** to get a first impression of how Crospi behaves.

    - **Step 2 — Developing:** Follow the **Developing** tutorials in order and learn how to develop in Crospi. The tutorials are presented in order of complexity, from easy high-level programming reusing libraries, to more low level programming that will require more expertise. 


??? note "Working in a Team :material-cursor-default-click:"

    If you are working as part of a team, we recommend assigning roles based on the diagram below. Each team member can then follow the tutorials relevant to their responsibilities.

    - **Step 1 — Common Foundation:** All team members should complete the **Templates**, **How to Run**, and **Examples** tutorials.

    - **Step 2 — Role-Specific Tutorials:** Click on the blocks in the diagram below to navigate to the tutorials associated with your role.
    With Crospi, team members can focus on the parts that matter to them—**not everyone needs to learn everything**.


## Structure of the tutorials:

!!! tip
    If you don't understand a term while going through the tutorials, try looking it up in the [glossary section](/markdown/extras/glossary/#glossary)


- **Developing → Applications:** Here you will learn how to reuse existing skill specifications from Python together with JSON configuration files.

- **Developing → Skills:** These tutorials introduce how to combine and reuse libraries of eTaSL Task Specifications from Python and JSON. The tools are similar to those used in application development, but the focus shifts to creating reusable, modular behaviors and composing multiple task specifications.

- **Developing → Task Specifications:** In these tutorials you will learn to create your own constraint-based tasks in eTaSL and learn how to implement complex, sensor-based, reactive robot behaviors that will be executed in real time.

- **Developing →  Plugins:** Only follow this step if your application requires functionality not provided by the existing plugins. These tutorials cover how to extend Crospi with plugins written in C++. You will need this only if, for example:

    * you want to integrate a robot that is not yet supported,
    * you require custom I/O handling (e.g., ZeroMQ instead of ROS 2 topics, or a specific ROS2 message type that is not yet supported),
    * you are an advanced user and require a solver different than qpOASES.

<figure>
  <div id="svg-container" style="width:100%;"></div>
  <figcaption>
    <h6 align="center">Click on the different blocks to jump to the corresponding tutorials.</h6>
  </figcaption>
</figure>


