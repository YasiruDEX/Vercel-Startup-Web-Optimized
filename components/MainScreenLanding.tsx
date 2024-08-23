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
import { ArrowUpIcon } from "@/components/Icons/icons"
import { Inter } from 'next/font/google';

export function MainScreenLanding() {
  return (
    <div className="flex flex-col min-h-dvh">

      <HeaderSection />

      <LandingIntro />

      <AboutSection />

      <ServicesSection />

      <ProjectsSection />

      <AwardsSection />

      <TeamSection />

      <EmailingSection />

      <BottomBanner />

      <ScrollToTop />

      <FooterSection />
      
    </div>
  );
}

function BottomBanner() {
  return (
    <section id="Banner" className="w-full bg-background">
      <div>
        <img
          src="/Bottom banner.png"
          alt="Banner"
          className="brightness-115 hidden md:block"
        />
      </div>
    </section>
  );
}

function ScrollToTop() {
  return (
    <div className="fixed bottom-4 right-4">
      <Button
        variant="ghost"
        size="icon"
        className="bg-primary/30 text-primary-foreground hover:bg-primary/30 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring backdrop-blur-lg"
        // onClick={() => window.scrollTo({ top: 0, behavior: "smooth" })}
      >
        <ArrowUpIcon className="w-6 h-6" />
        <span className="sr-only">Scroll to top</span>
      </Button>
    </div>
  );
}
