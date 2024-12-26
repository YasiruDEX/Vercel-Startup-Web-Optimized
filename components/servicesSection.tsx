import { useState } from "react";
import {
  FaAccusoft,
  FaRobot,
  FaCamera,
  FaCode,
  FaWifi,
  FaMicrochip,
  FaProjectDiagram,
} from "react-icons/fa";

import { IconType } from "react-icons";

interface ServiceCardProps {
  icon: IconType;
  title: string;
  description: string;
  technologies: string[];
}

const ServiceCard = ({ icon: Icon, title, description, technologies }: ServiceCardProps) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div
      className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert hover:scale-105"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      <Icon className="h-10 w-10" />
      <h3 className="text-lg font-bold">{title}</h3>
      {isHovered ? (
        <div className="text-sm text-muted-foreground text-center">
          <p className="font-bold">Technologies</p>
          <ul className="mt-2 list-disc list-inside">
            {technologies.map((tech, index) => (
              <li key={index}>{tech}</li>
            ))}
          </ul>
        </div>
      ) : (
        <p className="text-sm text-muted-foreground">{description}</p>
      )}
    </div>
  );
};

export default function ServicesSection() {
  const services = [
    {
      icon: FaAccusoft,
      title: "AI Automation",
      description:
        "AI-powered automation solutions that streamline operations and enhance efficiency.",
      technologies: [
        "Role automation",
        "Chatbots",
        "Model Training",
        "Fine-tuning",
      ],
    },
    {
      icon: FaRobot,
      title: "Robotics Solution",
      description:
        "Advanced robotic solutions that automate tasks and enhance productivity.",
      technologies: [
        "Industrial Robots",
        "Autonomous Navigation",
        "Kinematics",
        "Path Planning",
      ],
    },
    {
      icon: FaCamera,
      title: "Machine Vision Solutions",
      description:
        "Precision vision systems for automated inspection, identification, and control.",
      technologies: ["Object Detection", "Image Processing", "Camera Calibration"],
    },
    {
      icon: FaCode,
      title: "Software Development",
      description:
        "Custom software solutions designed to optimize performance and scalability.",
      technologies: ["Web Development", "API Integration", "Mobile Apps"],
    },
    {
      icon: FaWifi,
      title: "IoT Solutions",
      description:
        "Connected devices and systems that streamline operations and enhance user experiences.",
      technologies: ["Embedded Systems", "MQTT", "LoRaWAN", "Cloud Integration"],
    },
    {
      icon: FaProjectDiagram,
      title: "FPGA Solutions",
      description:
        "Specialized FPGA solutions for high-performance computing and custom hardware design.",
      technologies: [
        "VHDL/Verilog Programming",
        "RTL Design",
        "Simulation and Debugging",
        "High-Speed Data Processing",
      ],
    },
    {
      icon: FaMicrochip,
      title: "Machine Learning & AI",
      description:
        "Intelligent algorithms that drive data-driven decisions and automation.",
      technologies: [
        "Neural Networks",
        "NLP",
        "Model Deployment",
        "Data Preprocessing",
      ],
    },
  ];

  return (
    <section id="services" className="pt-20 pb-10 bg-muted flex justify-center">
      <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
        <div className="space-y-3 mx-auto max-w-[800px]">
          <h2 className="text-3xl font-bold font-sans tracking-tighter md:text-4xl/tight">
            What We Offer
          </h2>
          <p className="mx-auto max-w-[600px] text-muted-foreground md:text-md/relaxed lg:text-base/relaxed xl:text-md/relaxed pb-5">
            We offer a comprehensive suite of advanced technology solutions to
            help your business stay ahead in a rapidly evolving digital
            landscape.
          </p>
        </div>
        <div className="mx-auto grid max-w-5xl grid-cols-1 gap-6 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
          {services.map((service, index) => (
            <ServiceCard
              key={index}
              icon={service.icon}
              title={service.title}
              description={service.description}
              technologies={service.technologies}
            />
          ))}
        </div>
      </div>
    </section>
  );
}