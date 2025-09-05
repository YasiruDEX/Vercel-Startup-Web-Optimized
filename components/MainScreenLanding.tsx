"use client";
import { useState, useEffect, useRef } from "react";
import { Button } from "@/components/ui/button";
import AboutSection from "@/components/aboutSection";
import HeaderSection from "@/components/header";
import LandingIntro from "@/components/landingIntro";
import EmailingSection from "./emailingSection";
import ServicesSection from "./servicesSection";
import ProjectsSection from "./projectsSection";
import AwardsSection from "./awardsSection";
import TeamSection from "./teamSection";
import FooterSection from "./footerSection";
import DevelopmentShowcaseSection from "./developmentShowcaseSection";
import TestimonialSection from "./testimonialSection";
import LatestNewsSection from "./latestNewsSection";
import { ArrowUpIcon } from "@/components/Icons/icons";
import { Inter } from "next/font/google";
import { useDarkMode } from "@/components/darkModeProvider"; // Adjust path as necessary
import { BannerSlideshow } from "./bannerShow";


export function MainScreenLanding() {
  const [isMobile, setIsMobile] = useState(false);
  const [scrollY, setScrollY] = useState(0);
  const [windowHeight, setWindowHeight] = useState(0);
  const bannerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Function to check screen size and set window height
    const checkScreenSize = () => {
      setIsMobile(window.matchMedia('(max-width: 768px)').matches);
      setWindowHeight(window.innerHeight);
    };

    // Initial check
    checkScreenSize();

    // Add resize event listener
    window.addEventListener('resize', checkScreenSize);

    // Cleanup event listener on component unmount
    return () => window.removeEventListener('resize', checkScreenSize);
  }, []);

  useEffect(() => {
    const handleScroll = () => {
      setScrollY(window.scrollY);
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  // Calculate banner transform for parallax effect
  const bannerTransform = windowHeight > 0 ? Math.min(scrollY * 0.5, windowHeight) : 0;

  return (
    <div className={`relative min-h-screen custom-cursor overflow-x-hidden`}>
      <HeaderSection />
      
      {/* Fixed Banner Background */}
      <div 
        ref={bannerRef}
        className="fixed top-0 left-0 w-full h-screen z-0"
        style={{
          transform: `translateY(${bannerTransform}px)`,
          willChange: 'transform'
        }}
      >
        {isMobile ? <LandingIntro /> : <BannerSlideshow />}
      </div>

      {/* Content that stacks on top */}
      <div className="relative z-10">
        {/* Spacer to push content below viewport initially */}
        <div 
          className="w-full"
          style={{ height: `${windowHeight}px` }}
        />
        
        {/* Stacking Content Sections with rounded top corners */}
        <div className="bg-background relative rounded-t-3xl shadow-2xl">
          {!isMobile && (
            <div className="pt-8">
              <AboutSection />
            </div>
          )}
          <ServicesSection />
          <DevelopmentShowcaseSection />
          <TestimonialSection />
          <ProjectsSection />
          <AwardsSection />
          <LatestNewsSection />
          {/* <TeamSection /> */}
          <EmailingSection />
          <BottomBanner />
          <FooterSection />
        </div>
      </div>

      <ScrollToTop />
    </div>
  );
}

function BottomBanner() {
  const { darkMode, setDarkMode } = useDarkMode();
  return (
    <section id="Banner" className="w-full bg-background">
      <div>
        <img
          src={darkMode ? "/Bottom banner light.png" : "/Bottom banner.png"}
          alt="Banner"
          className="brightness-115 hidden md:block"
        />
      </div>
    </section>
  );
}

function ScrollToTop() {
  return (
    <div className="fixed bottom-4 left-4">
      <Button
        variant="ghost"
        size="icon"
        className="bg-primary/30 text-primary-foreground hover:bg-primary/30 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring backdrop-blur-lg"
        onClick={() => window.scrollTo({ top: 0, behavior: "smooth" })}
      >
        <ArrowUpIcon className="w-6 h-6" />
        <span className="sr-only">Scroll to top</span>
      </Button>
    </div>
  );
}
