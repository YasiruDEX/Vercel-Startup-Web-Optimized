"use client";
import { useState } from "react";
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
import { ArrowUpIcon } from "@/components/Icons/icons";
import { Inter } from "next/font/google";
import { useEffect } from "react";
import { useDarkMode } from "@/components/darkModeProvider"; // Adjust path as necessary
import { BannerSlideshow } from "./bannerShow";


export function MainScreenLanding() {
  const [isMobile, setIsMobile] = useState(false);

  useEffect(() => {
    // Function to check screen size
    const checkScreenSize = () => {
      setIsMobile(window.matchMedia('(max-width: 768px)').matches);
    };

    // Initial check
    checkScreenSize();

    // Add resize event listener
    window.addEventListener('resize', checkScreenSize);

    // Cleanup event listener on component unmount
    return () => window.removeEventListener('resize', checkScreenSize);
  }, []);

  return (
    <div className={`flex flex-col min-h-dv custom-cursor`}>
      <HeaderSection />
      {isMobile ? <LandingIntro /> : <BannerSlideshow />}
      {isMobile ? <></> : <AboutSection />}
      <ServicesSection />
      <ProjectsSection />
      <AwardsSection />
      {/* <TeamSection /> */}
      <EmailingSection />
      <BottomBanner />
      <ScrollToTop />
      <FooterSection />
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
