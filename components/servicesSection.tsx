import { useState } from "react";
import { Button } from "@/components/ui/button"

import { Bot, Zap, Cloud, Layers, FlaskRound, ServerCog } from "lucide-react";


interface ServiceCardProps {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  description: string;
  technologies: string[];
}

const ServiceCard = ({ icon: Icon, title, description, technologies }: ServiceCardProps) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div 
      className="group relative bg-card rounded-2xl border border-border p-6 transition-all duration-700 ease-in-out hover:scale-[1.025] hover:shadow-xl hover:border-primary/50 hover:bg-primary/5 cursor-pointer h-full flex flex-col hover:-translate-y-2"
      style={{
        objectFit: "cover",
        borderRadius: "10px",
    }}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      {/* Overlay + Glow */}
      <div className="absolute inset-0 bg-gradient-to-br from-transparent to-primary/5 group-hover:from-primary/5 group-hover:to-primary/10 transition-all duration-700 rounded-2xl pointer-events-none" />
      <div className="absolute inset-0 opacity-0 group-hover:opacity-100 transition duration-700 bg-gradient-to-r from-primary/10 via-transparent to-primary/10 blur-sm rounded-2xl pointer-events-none" />

      {/* Icon */}
      <div className="relative z-10 flex justify-center mb-6">
        <div className="relative">
          <div className="absolute inset-0 bg-primary/20 rounded-full blur-xl group-hover:blur-2xl transition-all duration-700 group-hover:scale-125 group-hover:bg-primary/30" />
          <div className="relative bg-gradient-to-br from-primary to-primary/80 p-4 rounded-full transition-transform duration-500 group-hover:rotate-12 group-hover:scale-110 shadow-lg group-hover:shadow-primary/30" style={{
                  objectFit: "cover",
                  borderRadius: "50px",
              }}>
            <Icon className="h-8 w-8 text-white" />
          </div>
        </div>
      </div>

      {/* Title */}
      <h3 className="text-xl font-semibold text-center mb-4 transition group-hover:text-primary group-hover:scale-105">
        {title}
      </h3>

      {/* Description */}
      <p className="text-muted-foreground text-center text-sm leading-relaxed mb-6 transition group-hover:text-foreground">
        {description}
      </p>

      {/* Technologies */}
      <div className={`transition-all duration-700 ease-in-out ${isHovered ? 'opacity-100 max-h-40 translate-y-0' : 'opacity-0 max-h-0 translate-y-4'} overflow-hidden`}>
        <div className="border-t border-border pt-4">
          <p className="text-xs font-semibold text-primary text-center mb-3">Core Technologies</p>
          <div className="flex flex-wrap justify-center gap-2">
            {technologies.map((tech, index) => (
              <span
                key={index}
                className="inline-block px-3 py-1 text-xs bg-secondary text-secondary-foreground rounded-full border border-secondary/30 transition-all duration-300 hover:bg-primary/10 hover:border-primary/50 hover:scale-110"
                style={{ transitionDelay: `${index * 100}ms`,objectFit: "cover",
                  borderRadius: "30px", }}
              >
                {tech}
              </span>
            ))}
          </div>
        </div>
      </div>

      {/* CTA */}
      <div className="flex justify-center mt-6">
        <Button 
          variant="outline" 
          size="sm"
          className="text-xs transition-all duration-500 hover:bg-primary hover:text-primary-foreground hover:scale-105 hover:shadow-md border-primary/30 opacity-0 group-hover:opacity-100 group-hover:translate-y-0 translate-y-3"
        >
          Learn More
        </Button>
      </div>
    </div>
  );
};


export default function ServicesSection() {
  const services = [
    {
      icon: Bot,
      title: "AI-Powered Solutions",
      description: "Custom ML models, LLMs, real-time pipelines, and computer vision systems.",
      technologies: ["PyTorch", "TensorFlow", "OpenCV", "Transformers"],
    },
    {
      icon: Zap,
      title: "Embedded Systems & IoT",
      description: "Smart sensors, LoRa, ESP32 systems, and real-time wireless solutions.",
      technologies: ["ESP32", "LoRa", "MQTT", "Arduino"],
    },
    {
      icon: Layers,
      title: "Robotics & Autonomous Systems",
      description: "SLAM, path planning, ROS2-based robots for automation and navigation.",
      technologies: ["ROS2", "SLAM", "OpenCV", "Gazebo"],
    },
    {
      icon: Cloud,
      title: "Cloud & App Development",
      description: "Design and development of scalable web and mobile applications with cloud integration.",
      technologies: [
        "Node.js",
        "React",
        "Flutter",
        "Next.js",
        "Express.js",
        "MongoDB",
        "MySQL",
        "React Native",
        "Flask",
        "Spring Boot",
      ],
    }, 
    {
      icon: ServerCog,
      title: "DevOps & Infrastructure",
      description: "CI/CD automation, containerization, cloud infrastructure, and monitoring solutions for modern development workflows.",
      technologies: [
        "Docker",
        "GitHub Actions",
        "Jenkins",
        "AWS",
        "GCP",
      ],
    },
    {
      icon: FlaskRound,
      title: "AI/ML Research & Consulting",
      description: "Model fine-tuning, system design, research support, ethical AI strategies.",
      technologies: ["Research", "Consulting", "Ethics", "Strategy"],
    },
  ];

  return (
    <div className="min-h-screen bg-background">
      <section id="services" className="py-20 bg-gradient-to-br from-background via-muted/10 to-background relative overflow-hidden">
        
        {/* Background blur circles */}
        <div className="absolute inset-0">
          <div className="absolute top-1/4 left-1/3 w-64 h-64 bg-primary/10 rounded-full blur-3xl animate-pulse" />
          <div className="absolute bottom-1/4 right-1/3 w-72 h-72 bg-primary/10 rounded-full blur-3xl animate-pulse delay-500" />
          <div className="absolute top-1/2 left-1/2 w-60 h-60 bg-secondary/10 rounded-full blur-2xl animate-pulse delay-700 transform -translate-x-1/2 -translate-y-1/2" />
        </div>

        <div className="container mx-auto px-4 relative z-10">
          <div className="text-center mb-16">
            <h2 className="text-4xl font-bold tracking-tight mb-4">Our Services</h2>
            <p className="text-muted-foreground max-w-2xl mx-auto text-lg">
              Explore a suite of cutting-edge solutions designed to drive innovation and digital transformation.
            </p>
          </div>

          {/* Services Grid */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {services.map((service, index) => (
              <div
                key={index}
                className="animate-fade-in"
                style={{ animationDelay: `${index * 200}ms` }}
              >
                <ServiceCard {...service} />
              </div>
            ))}
          </div>

          {/* CTA Button */}
          <div className="text-center mt-16">
            <a
              href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27d%20like%20to%20connect%20with%20the%20Aura%20Digital%20Labs%20team%20for%20a%20discussion."
              target="_blank"
              rel="noopener noreferrer"
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-gray-700 to-black hover:from-gray-600 hover:to-gray-800 text-white font-semibold px-8 py-3 rounded-full tracking-wide shadow-md hover:shadow-lg transition"
                style={{
                  objectFit: "cover",
                  borderRadius: "30px",
              }}
              >
                Book a Free Call
              </Button>
            </a>
          </div>
        </div>
      </section>
    </div>
  );
}
