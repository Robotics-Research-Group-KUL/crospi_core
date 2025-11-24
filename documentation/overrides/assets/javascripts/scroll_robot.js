// document.addEventListener("DOMContentLoaded", () => {
//     const robot = document.querySelector(".scroll-robot");
//     if (!robot) return;
  
//     const speed = 0.25; // tweak movement strength
  
//     const update = () => {
//       const scrollY = window.scrollY;
//       const offset = scrollY * speed;
  
//       robot.style.transform =
//         `translateX(-50%) translateY(${offset}px)`;
  
//       requestAnimationFrame(update);
//     };
  
//     requestAnimationFrame(update);
//   });
  

  document.addEventListener("DOMContentLoaded", () => {
    const robot = document.querySelector(".scroll-robot-background img");
    const speed = 0.25;
  
    const update = () => {
      const offset = window.scrollY * speed;
      robot.style.transform = `translateY(${offset}px)`; // adjust Y offset only
      requestAnimationFrame(update);
    };
  
    requestAnimationFrame(update);
  });