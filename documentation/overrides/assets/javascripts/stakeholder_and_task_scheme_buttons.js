fetch('/images/stakeholders_and_task_scheme.svg')
  .then(res => res.text())
  .then(svg => {
    const container = document.getElementById('svg-container');
    container.innerHTML = svg;
    - 'markdown/advanced_tutorials/robot_drivers.md'
    - 'markdown/advanced_tutorials/io_handlers.md'
    - 'markdown/advanced_tutorials/solvers.md'
    // Define clickable rects and their URLs
    const routes = {
      'rect1557-6': '/markdown/advanced_tutorials/plugins_intro/#plugins-intro', //Plugins block cathegory
        'rect878-04': '/markdown/advanced_tutorials/robot_drivers/#robot-drivers', //Robot drivers button
        'rect878-6-8': '/markdown/advanced_tutorials/io_handlers/#io-handlers', //IO-handlers button
        'rect878-6-5-2': '/markdown/advanced_tutorials/solvers/#solvers', //Solvers button

      'rect1557': '/markdown/basic_tutorials/applications_intro/#applications-intro', //Application block cathegory
        'rect878': '/markdown/basic_tutorials/setup_configuration/#setup-configuration', //Setup configuration button
        'rect878-6': '/markdown/basic_tutorials/application_orchestration/#application-orchestration', //Orchestration button
        'rect878-6-5': '/markdown/basic_tutorials/application_launch_files/#application-launch-files', //Launch file button

      'rect1557-1': '/markdown/basic_tutorials/skills_intro/#skills-intro', //Skill specifications block cathegory
        'rect878-0': '/markdown/basic_tutorials/skill_orchestration/#skill-orchestration', //Orchestration button
        'rect878-0-6': '/markdown/basic_tutorials/skill_specification_libraries/#skill-specification-libraries', //Skill specification libraries button

      'rect1557-1-4': '/markdown/basic_tutorials/task_specifications_intro/#task-specifications-intro', //Task specifications block cathegory
        'rect878-0-68': '/markdown/basic_tutorials/specification_of_constraints_in_etasl/#specification-of-constraints-in-etasl', //Specification of constraints button
        'rect878-0-6-2': '/markdown/basic_tutorials/creating_task_specification_libraries/#creating-task-specification-libraries', //Task specification libraries button
    };

    // Loop over all IDs in the routes object
    Object.entries(routes).forEach(([id, url]) => {
      const el = container.querySelector(`#${CSS.escape(id)}`);
      if (el) {
        // Add shared class for CSS styling
        el.classList.add('svg-button');

        // Enable pointer events and bind click
        el.style.pointerEvents = 'all';
        el.addEventListener('click', () => { window.location.href = url; });

        // Make text inside the same parent pass clicks through
        const parent = el.parentNode;
        if (parent) {
          parent.querySelectorAll('text').forEach(t => {
            t.style.pointerEvents = 'none';
          });
        }
      } else {
        console.warn('SVG element not found:', id);
      }
    });
  })
  .catch(err => console.error('Failed to load SVG:', err));
