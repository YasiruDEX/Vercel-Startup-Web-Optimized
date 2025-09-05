"use client";
import { useDarkMode } from "@/components/darkModeProvider";
import { useState, useEffect } from "react";

export default function DevelopmentShowcaseSection() {
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

    const section = document.getElementById("development-showcase");
    if (section) observer.observe(section);

    return () => {
      if (section) observer.unobserve(section);
    };
  }, []);

  return (
    <section
      id="development-showcase"
      className={`py-16 md:py-24 transition-colors duration-300 ${
        darkMode ? "bg-gray-900" : "bg-gray-50"
      }`}
    >
      <div className="container mx-auto px-4 sm:px-6 lg:px-8 max-w-7xl">
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 lg:gap-16 items-center">
          
          {/* Left side - Image/Video */}
          <div className="relative order-2 lg:order-1">
            <div className="relative overflow-hidden rounded-2xl shadow-2xl bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-900">
              {/* Development environment mockup */}
              <div className="aspect-[4/3] flex items-center justify-center p-8">
                <div className="w-full h-full bg-gray-900 rounded-lg p-4 shadow-inner">
                  {/* Simulated code editor */}
                  <div className="flex items-center space-x-2 mb-4">
                    <div className="w-3 h-3 bg-gray-500 rounded-full"></div>
                    <div className="w-3 h-3 bg-gray-400 rounded-full"></div>
                    <div className="w-3 h-3 bg-gray-600 rounded-full"></div>
                  </div>
                  <div className="space-y-2">
                    <div className="h-2 bg-gray-400 rounded w-3/4"></div>
                    <div className="h-2 bg-gray-500 rounded w-1/2"></div>
                    <div className="h-2 bg-gray-400 rounded w-2/3"></div>
                    <div className="h-2 bg-gray-600 rounded w-5/6"></div>
                    <div className="h-2 bg-gray-500 rounded w-1/3"></div>
                    <div className="h-2 bg-gray-400 rounded w-4/5"></div>
                  </div>
                  <div className="mt-6 text-gray-400 text-xs font-mono">
                    $ npm run build<br/>
                    ✓ Build successful
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Right side - Content */}
          <div className="order-1 lg:order-2">
            <div className="space-y-8">
              
              {/* Main content */}
              <div className="space-y-6">
                <div className="space-y-4">
                  <h2
                    className={`text-3xl md:text-4xl lg:text-5xl font-bold leading-tight transition-all duration-700 ${
                      isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                    } ${darkMode ? "text-white" : "text-gray-900"}`}
                  >
                    Our Development Process
                  </h2>
                  <p
                    className={`text-lg md:text-xl leading-relaxed transition-all duration-700 delay-100 ${
                      isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                    } ${darkMode ? "text-gray-300" : "text-gray-700"}`}
                  >
                    We employ a comprehensive and holistic approach that ensures all aspects and stages are thoughtfully and thoroughly addressed.
                  </p>
                </div>

                {/* <div className="space-y-4">
                  <div
                    className={`transition-all duration-700 delay-200 ${
                      isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                    }`}
                  >
                    <div className={`p-6 rounded-xl border ${darkMode ? "bg-gray-800/50 border-gray-700" : "bg-white/80 border-gray-200"} backdrop-blur-sm hover:shadow-lg transition-shadow duration-300`}>
                      <h4 className={`font-semibold mb-3 ${darkMode ? "text-white" : "text-gray-900"}`}>
                        Development Excellence
                      </h4>
                      <p className={`text-sm leading-relaxed ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                        Our state-of-the-art development environment enables rapid prototyping, seamless collaboration, and robust testing across multiple platforms and technologies.
                      </p>
                    </div>
                  </div>

                  <div
                    className={`transition-all duration-700 delay-300 ${
                      isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                    }`}
                  >
                    <div className={`p-6 rounded-xl border ${darkMode ? "bg-gray-800/50 border-gray-700" : "bg-white/80 border-gray-200"} backdrop-blur-sm hover:shadow-lg transition-shadow duration-300`}>
                      <h4 className={`font-semibold mb-3 ${darkMode ? "text-white" : "text-gray-900"}`}>
                        Quality Assurance
                      </h4>
                      <p className={`text-sm leading-relaxed ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                        Rigorous testing protocols and continuous integration ensure that every line of code meets our high standards for performance, security, and reliability.
                      </p>
                    </div>
                  </div>
                </div> */}
              </div>

              {/* Statistics */}
              <div
                className={`transition-all duration-700 delay-500 ${
                  isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-4"
                }`}
              >
                <div className={`p-8 rounded-2xl ${darkMode ? "bg-gradient-to-br from-gray-800 to-gray-900" : "bg-gradient-to-br from-gray-50 to-white"} border ${darkMode ? "border-gray-700" : "border-gray-200"} shadow-lg`}>
                  {/* 100% Client Satisfaction */}
                  <div className="mb-8">
                    <div className={`text-7xl md:text-8xl font-black ${darkMode ? "text-white" : "text-gray-900"} leading-none`}>
                      100%
                    </div>
                    <div className={`text-sm md:text-base font-bold tracking-widest uppercase mt-2 ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                      Client Satisfaction
                    </div>
                  </div>

                  {/* Additional stats grid */}
                  <div className="grid grid-cols-3 gap-6 pt-6 border-t border-gray-200 dark:border-gray-700">
                    <div className="text-center">
                      <div className={`text-2xl md:text-3xl font-bold ${darkMode ? "text-white" : "text-gray-900"}`}>
                        97+
                      </div>
                      <div className={`text-xs md:text-sm font-medium mt-1 ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                        Projects
                      </div>
                    </div>
                    <div className="text-center">
                      <div className={`text-2xl md:text-3xl font-bold ${darkMode ? "text-white" : "text-gray-900"}`}>
                        24/7
                      </div>
                      <div className={`text-xs md:text-sm font-medium mt-1 ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                        Support
                      </div>
                    </div>
                    <div className="text-center">
                      <div className={`text-2xl md:text-3xl font-bold ${darkMode ? "text-white" : "text-gray-900"}`}>
                        5+
                      </div>
                      <div className={`text-xs md:text-sm font-medium mt-1 ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                        Years
                      </div>
                    </div>
                  </div>

                  {/* Trust indicators */}
                  <div className="mt-6 pt-6 border-t border-gray-200 dark:border-gray-700">
                    <div className="flex items-center justify-center space-x-6">
                      <div className="flex items-center space-x-2">
                        <div className={`w-3 h-3 rounded-full ${darkMode ? "bg-gray-400" : "bg-gray-600"}`}></div>
                        <span className={`text-xs font-medium ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                          Active Development
                        </span>
                      </div>
                      <div className="flex items-center space-x-2">
                        <div className={`w-3 h-3 rounded-full ${darkMode ? "bg-gray-400" : "bg-gray-600"}`}></div>
                        <span className={`text-xs font-medium ${darkMode ? "text-gray-400" : "text-gray-600"}`}>
                          Enterprise Ready
                        </span>
                      </div>
                    </div>
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
