"use client";
import { useDarkMode } from "@/components/darkModeProvider";
import { useState, useEffect } from "react";

export default function TestimonialSection() {
  const { darkMode } = useDarkMode();
  const [isInView, setIsInView] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsInView(true);
        }
      },
      { threshold: 0.3 }
    );

    const section = document.getElementById("testimonial-section");
    if (section) observer.observe(section);

    return () => {
      if (section) observer.unobserve(section);
    };
  }, []);

  return (
    <section
      id="testimonial-section"
      className={`py-16 md:py-24 transition-colors duration-300 ${
        darkMode ? "bg-gray-900" : "bg-gray-800"
      }`}
    >
      <div className="container mx-auto px-4 sm:px-6 lg:px-8 max-w-7xl">
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 lg:gap-16 items-center">
          
          {/* Left side - Professional Image */}
          <div className="relative order-1 lg:order-1">
            <div className="relative overflow-hidden rounded-2xl">
              <img
                src="https://images.unsplash.com/photo-1560472354-b33ff0c44a43?ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D&auto=format&fit=crop&w=1000&q=80"
                alt="Professional Business Meeting"
                className="w-full h-auto object-cover transition-transform duration-700 hover:scale-105"
              />
              
              {/* Large quote mark overlay */}
              <div className="absolute top-8 right-8 md:top-12 md:right-12">
          <div className="text-6xl md:text-8xl font-serif text-white opacity-20">
            &ldquo;
          </div>
              </div>
            </div>
          </div>

          {/* Right side - Testimonial Content */}
          <div className="order-2 lg:order-2 text-white">
            <div className="space-y-8">
              
              {/* Quote Mark */}
              <div
                className={`transition-all duration-700 ${
                  isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                }`}
              >
                <div className="text-6xl md:text-8xl font-serif text-orange-400 leading-none">
                  &ldquo;
                </div>
              </div>

              {/* Main Quote */}
              <div className="space-y-6">
                <blockquote
                  className={`text-xl md:text-2xl lg:text-3xl leading-relaxed font-light transition-all duration-700 delay-200 ${
                    isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                  }`}
                >
                  AURA Digital Labs represents the pinnacle of technological innovation and client dedication. Our commitment to excellence has consistently delivered transformative solutions that drive business success across industries.
                </blockquote>

                {/* Pagination dots */}
                <div
                  className={`flex space-x-2 transition-all duration-700 delay-400 ${
                    isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                  }`}
                >
                  <div className="w-2 h-2 bg-white rounded-full"></div>
                  <div className="w-8 h-2 bg-orange-400 rounded-full"></div>
                  <div className="w-2 h-2 bg-white/30 rounded-full"></div>
                </div>
              </div>

              {/* Author Information */}
              <div
                className={`flex items-center space-x-4 transition-all duration-700 delay-600 ${
                  isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                }`}
              >
                <div className="relative">
                  <img
                    src="/Aura Logo-01.png"
                    alt="AURA Digital Labs Logo"
                    className="w-16 h-16 rounded-full bg-white p-2 shadow-lg object-contain"
                  />
                </div>
                <div>
                  <div className="font-semibold text-lg md:text-xl text-white">
                    Executive Board
                  </div>
                  <div className="text-orange-400 font-medium text-sm md:text-base tracking-wider uppercase">
                    AURA Digital Labs
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
