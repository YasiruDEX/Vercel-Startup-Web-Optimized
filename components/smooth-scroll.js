'use client';
import { useEffect } from 'react';

const smoothScrollTo = (targetElement, duration = 600) => {
  const start = window.scrollY;
  const targetPosition = targetElement.getBoundingClientRect().top + window.scrollY;
  const distance = targetPosition - start;
  let startTime = null;

  const scrollAnimation = (currentTime) => {
    if (startTime === null) startTime = currentTime;
    const timeElapsed = currentTime - startTime;
    const progress = Math.min(timeElapsed / duration, 1); // Ensure it doesn't go above 1

    window.scrollTo(0, start + distance * easeInOutQuad(progress));
    if (timeElapsed < duration) {
      requestAnimationFrame(scrollAnimation);
    }
  };

  const easeInOutQuad = (t) => {
    return t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
  };

  requestAnimationFrame(scrollAnimation);
};

const SmoothScroll = () => {
  useEffect(() => {
    const links = document.querySelectorAll('a[href^="#"]');
  
    links.forEach(link => {
      link.addEventListener('click', (e) => {
        e.preventDefault();
        const targetId = link.getAttribute('href').substring(1);
        const targetElement = document.getElementById(targetId);
        if (targetElement) {
          smoothScrollTo(targetElement);
        }
      });
    });

    // Clean up event listeners on component unmount
    return () => {
      links.forEach(link => link.removeEventListener('click', () => {}));
    };
  }, []);

  return null;
};

export default SmoothScroll;
