"use client";
import { useEffect } from "react";

const ClickEffect = () => {
  useEffect(() => {
    const handleClick = (e: { clientX: any; clientY: any; }) => {
      const ripple = document.createElement("div");
      ripple.className = "click-ripple";
      ripple.style.left = `${e.clientX}px`;
      ripple.style.top = `${e.clientY}px`;
      document.body.appendChild(ripple);

      setTimeout(() => {
        ripple.remove();
      }, 600); // match animation duration
    };

    window.addEventListener("click", handleClick);
    return () => window.removeEventListener("click", handleClick);
  }, []);

  return null;
};

export default ClickEffect;