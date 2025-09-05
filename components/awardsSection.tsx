"use client";
import { useState, useEffect } from 'react';
import { Trophy, Award, Star, ChevronDown, ChevronUp } from 'lucide-react';

export default function AwardsSection() {
  const [showAll, setShowAll] = useState(false);
  const [isInView, setIsInView] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsInView(true);
        }
      },
      { threshold: 0.1 }
    );

    const section = document.getElementById('awards');
    if (section) {
      observer.observe(section);
    }

    return () => {
      if (section) {
        observer.unobserve(section);
      }
    };
  }, []);

  const awards = [
    {
      title: "Championship - SLIoT Challenge",
      project: "Project Hydrolink",
      description: "All island Internet of Things competition",
      type: "championship",
      year: "2023"
    },
    {
      title: "Championship - Sri Lanka Circuit Challenge",
      project: "Project Steer-Safe",
      description: "IEEE Challenge Sphere",
      type: "championship",
      year: "2023"
    },
    {
      title: "Championship - Sri Lanka Arduino Challenge",
      project: "Project Replace",
      description: "IEEE Challenge sphere",
      type: "championship",
      year: "2022"
    },
    {
      title: "1st Runnersup - Brainstorm",
      project: "Project Steer-Safe",
      description: "Healthcare Innovation Competition",
      type: "runner-up",
      year: "2023"
    },
    {
      title: "1st Runnersup - Sri Lanka Circuit Challenge",
      project: "Project BlindGuide",
      description: "IEEE Challenge Sphere",
      type: "runner-up",
      year: "2022"
    },
    {
      title: "1st Runnersup - Sri Lanka AI Challenge",
      project: "Project ElectoBot",
      description: "IEEE Challenge Sphere",
      type: "runner-up",
      year: "2022"
    },
    {
      title: "1st Runnersup - Aurora",
      project: "Project Face Canvas",
      description: "AI Ideathlon",
      type: "runner-up",
      year: "2021"
    },
  ];

  const getIcon = (type: string) => {
    switch (type) {
      case 'championship':
        return <Trophy className="w-6 h-6" />;
      case 'runner-up':
        return <Award className="w-6 h-6" />;
      default:
        return <Star className="w-6 h-6" />;
    }
  };

  return (
    <section
      id="awards"
      className="py-20 lg:py-32 bg-gray-50"
    >
      <div className="max-w-7xl mx-auto px-6 lg:px-8">
        {/* Header */}
        <div
          className={`text-center mb-16 transition-all duration-700 ${
            isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
          }`}
        >
          <div className="inline-flex items-center justify-center w-16 h-16 bg-gray-900 rounded-2xl mb-6">
            <Trophy className="w-8 h-8 text-white" />
          </div>
          <h2 className="text-4xl lg:text-5xl xl:text-6xl font-bold text-gray-900 mb-6">
            Honors & <span className="text-gray-600">Awards</span>
          </h2>
          <p className="max-w-3xl mx-auto text-lg text-gray-600 leading-relaxed">
            Our commitment to innovation and excellence has been recognized through prestigious awards 
            and achievements across multiple technology competitions and industry challenges.
          </p>
          
          {/* Achievement Highlights */}
          {/* <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mt-12 max-w-4xl mx-auto">
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-200">
              <div className="text-3xl font-bold text-gray-900 mb-2">7+</div>
              <div className="text-gray-600 font-medium">Total Awards</div>
            </div>
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-200">
              <div className="text-3xl font-bold text-gray-900 mb-2">3</div>
              <div className="text-gray-600 font-medium">Championships</div>
            </div>
            <div className="bg-white rounded-xl p-6 shadow-sm border border-gray-200">
              <div className="text-3xl font-bold text-gray-900 mb-2">4</div>
              <div className="text-gray-600 font-medium">Runner-up Positions</div>
            </div>
          </div> */}
        </div>

        {/* Awards Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
          {(showAll ? awards : awards.slice(0, 4)).map((award, index) => (
            <div
              key={index}
              className={`group bg-white border border-gray-200 rounded-2xl p-8 hover:shadow-xl hover:border-gray-300 transition-all duration-500 hover:-translate-y-2 ${
                isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
              }`}
              style={{ transitionDelay: `${index * 100}ms` }}
            >
              <div className="flex items-start space-x-6">
                
                {/* Icon & Badge */}
                <div className="flex-shrink-0">
                  <div className="w-16 h-16 bg-gray-100 rounded-2xl flex items-center justify-center text-gray-700 group-hover:bg-gray-900 group-hover:text-white transition-all duration-300">
                    {getIcon(award.type)}
                  </div>
                  <div className="mt-3 text-center">
                    <span className="text-xs font-semibold text-gray-500 bg-gray-100 px-2 py-1 rounded-full">
                      {award.year}
                    </span>
                  </div>
                </div>

                {/* Content */}
                <div className="flex-1 min-w-0">
                  <div className="flex items-start justify-between mb-4">
                    <div className={`px-3 py-1 rounded-full text-xs font-semibold ${
                      award.type === 'championship' 
                        ? 'bg-amber-50 text-amber-700 border border-amber-200' 
                        : 'bg-blue-50 text-blue-700 border border-blue-200'
                    }`}>
                      {award.type === 'championship' ? '🏆 Champion' : '🥈 Runner-up'}
                    </div>
                  </div>
                  
                  <h3 className="text-xl font-bold text-gray-900 mb-3 group-hover:text-gray-700 transition-colors duration-300 leading-tight">
                    {award.title}
                  </h3>
                  
                  <div className="space-y-3">
                    <div className="flex items-center space-x-2">
                      <div className="w-2 h-2 bg-gray-400 rounded-full"></div>
                      <span className="text-sm font-semibold text-gray-800">
                        Project: {award.project}
                      </span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <div className="w-2 h-2 bg-gray-400 rounded-full"></div>
                      <span className="text-sm text-gray-600">
                        {award.description}
                      </span>
                    </div>
                  </div>

                  {/* Progress bar decoration */}
                  <div className="mt-4 w-full bg-gray-100 rounded-full h-1">
                    <div className={`h-1 rounded-full transition-all duration-1000 group-hover:w-full ${
                      award.type === 'championship' ? 'bg-amber-400 w-4/5' : 'bg-blue-400 w-3/5'
                    }`}></div>
                  </div>
                </div>
              </div>
            </div>
          ))}
        </div>

        {/* Show More/Less Button */}
        <div 
          className={`text-center mt-12 transition-all duration-700 delay-500 ${
            isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
          }`}
        >
          <button
            onClick={() => setShowAll(!showAll)}
            className="inline-flex items-center px-8 py-4 bg-gray-900 text-white font-semibold rounded-xl hover:bg-gray-800 transition-all duration-300 hover:shadow-lg hover:scale-105 border border-gray-900"
          >
            {showAll ? (
              <>
                Show Less Awards
                <ChevronUp className="ml-2 w-5 h-5" />
              </>
            ) : (
              <>
                View All {awards.length} Awards
                <ChevronDown className="ml-2 w-5 h-5" />
              </>
            )}
          </button>
          
          {/* Additional Info */}
          <p className="mt-4 text-sm text-gray-500 max-w-2xl mx-auto">
            These achievements reflect our team&apos;s dedication to pushing boundaries in technology, 
            innovation, and creating solutions that make a real impact.
          </p>
        </div>

        {/* Recognition Timeline */}
        <div 
          className={`mt-20 bg-white rounded-3xl p-8 lg:p-12 border border-gray-200 shadow-sm transition-all duration-700 delay-700 ${
            isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
          }`}
        >
          <div className="text-center mb-8">
            <h3 className="text-2xl lg:text-3xl font-bold text-gray-900 mb-4">
              Recognition Timeline
            </h3>
            <p className="text-gray-600">
              Our journey of excellence across multiple technology domains
            </p>
          </div>
          
          <div className="grid grid-cols-1 md:grid-cols-4 gap-6">
            <div className="text-center p-6 bg-gradient-to-br from-gray-50 to-gray-100 rounded-2xl">
              <div className="text-2xl font-bold text-gray-900 mb-2">2021</div>
              <div className="text-sm text-gray-600">AI Innovation</div>
              <div className="text-xs text-gray-500 mt-1">1 Award</div>
            </div>
            <div className="text-center p-6 bg-gradient-to-br from-gray-50 to-gray-100 rounded-2xl">
              <div className="text-2xl font-bold text-gray-900 mb-2">2022</div>
              <div className="text-sm text-gray-600">Circuit & AI Challenges</div>
              <div className="text-xs text-gray-500 mt-1">3 Awards</div>
            </div>
            <div className="text-center p-6 bg-gradient-to-br from-gray-50 to-gray-100 rounded-2xl">
              <div className="text-2xl font-bold text-gray-900 mb-2">2023</div>
              <div className="text-sm text-gray-600">IoT & Healthcare</div>
              <div className="text-xs text-gray-500 mt-1">3 Awards</div>
            </div>
            <div className="text-center p-6 bg-gradient-to-br from-blue-50 to-blue-100 rounded-2xl border border-blue-200">
              <div className="text-2xl font-bold text-blue-900 mb-2">2024+</div>
              <div className="text-sm text-blue-700">Expanding Horizons</div>
              <div className="text-xs text-blue-600 mt-1">More to Come</div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}