// scroll-robot-arm.js
// Animates the inline SVG robotic arm joints based on scroll position.
// Uses requestAnimationFrame for smoothness and minimal overhead.

document.addEventListener("DOMContentLoaded", () => {
    const container = document.getElementById("robot-scroll-section");
    const shoulder = document.getElementById("joint-shoulder");
    const elbow = document.getElementById("joint-elbow");
    const wrist = document.getElementById("joint-wrist");
    const gripper = document.getElementById("gripper");
  
    if (!container || !shoulder || !elbow || !wrist || !gripper) return;
  
    // Configuration: angle ranges (degrees)
    const ranges = {
      shoulder: { min: -18, max: 18 }, // rotate shoulder
      elbow:    { min: -35, max: 35 }, // rotate elbow (relative)
      wrist:    { min: -20, max: 20 }, // wrist tilt
      gripper:  { min: 0, max: 18 }    // how far gripper fingers open (px offset)
    };
  
    // Helper: map number from [a,b] -> [c,d]
    const map = (v, a, b, c, d) => {
      const t = Math.max(0, Math.min(1, (v - a) / (b - a)));
      return c + (d - c) * t;
    };
  
    // Calculate progress of container through viewport (0..1)
    function viewportProgress(el) {
      const rect = el.getBoundingClientRect();
      const winH = window.innerHeight || document.documentElement.clientHeight;
      // start when top enters bottom of viewport, end when bottom crosses top
      const start = winH * 0.9; // animation start point
      const end = - (rect.height * 0.2); // end point (when mostly scrolled past)
      // compute a normalized progress value
      const raw = map(rect.top, start, end, 0, 1);
      return Math.max(0, Math.min(1, raw));
    }
  
    // Apply transforms based on progress
    function applyTransforms(progress) {
      // compute angles
      const sAng = map(progress, 0, 1, ranges.shoulder.min, ranges.shoulder.max);
      const eAng = map(progress, 0, 1, ranges.elbow.min, ranges.elbow.max);
      const wAng = map(progress, 0, 1, ranges.wrist.min, ranges.wrist.max);
      const gOff = map(progress, 0, 1, ranges.gripper.min, ranges.gripper.max);
  
      // set CSS transforms on groups (works with transform-box: fill-box)
      shoulder.style.transform = `rotate(${sAng}deg)`;
      elbow.style.transform = `rotate(${eAng}deg)`;
      wrist.style.transform = `rotate(${wAng}deg)`;
  
      // gripper: open by translating left & right finger rectangles inside gripper group.
      // The gripper group itself moves subtly for effect.
      gripper.style.transform = `translateY(${gOff * 0.05}px)`; // small vertical nudge
  
      // find the two rect children (fingers) and offset them
      const fingers = gripper.querySelectorAll("rect");
      if (fingers && fingers.length >= 2) {
        // left finger moves left, right finger moves right
        fingers[0].style.transform = `translateX(${-gOff}px)`;
        fingers[1].style.transform = `translateX(${gOff}px)`;
        // hint: set transform-box for these rects via CSS if needed (done globally)
        fingers[0].style.transition = "transform 0.12s linear";
        fingers[1].style.transition = "transform 0.12s linear";
      }
    }
  
    // RAF loop
    let ticking = false;
    function loop() {
      if (!ticking) {
        ticking = true;
        requestAnimationFrame(() => {
          const p = viewportProgress(container);
          applyTransforms(p);
          ticking = false;
        });
      }
      // keep loop running to update as user scrolls
      requestAnimationFrame(loop);
    }
  
    // Kick off
    requestAnimationFrame(loop);
  });
  