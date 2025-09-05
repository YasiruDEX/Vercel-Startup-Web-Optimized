"use client";

import { useState, useRef } from "react";
import { Button } from "@/components/ui/button";
import { motion, useInView } from "framer-motion";
import { Bot, Zap, Cloud, Layers, FlaskRound, ServerCog } from "lucide-react";
import Link from "next/link";


interface ServiceCardProps {
  icon: React.ComponentType<{ className?: string }>;
  title: string;
  description: string;
  technologies: string[];
  link: string;
}

const ServiceCard = ({ icon: Icon, title, description, technologies, link, index }: ServiceCardProps & { index: number }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <motion.div
      initial={{ opacity: 0, y: 50, scale: 0.9 }}
      whileInView={{ 
        opacity: 1, 
        y: 0, 
        scale: 1,
        transition: {
          duration: 0.8,
          delay: index * 0.15,
          ease: [0.25, 0.46, 0.45, 0.94], // Custom bezier for smooth animation
        }
      }}
      whileHover={{ 
        y: -8, 
        scale: 1.02,
        transition: { duration: 0.3, ease: "easeOut" }
      }}
      viewport={{ once: true, margin: "-100px" }}
      className="group relative bg-card/80 backdrop-blur-sm rounded-3xl border border-border/50 p-8 cursor-pointer h-full flex flex-col hover:border-primary/30 hover:bg-card/90 transition-all duration-500"
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      {/* Enhanced glow effect */}
      <motion.div 
        className="absolute inset-0 bg-gradient-to-br from-primary/5 via-transparent to-primary/10 rounded-3xl opacity-0 group-hover:opacity-100 transition-all duration-500"
        initial={false}
        animate={isHovered ? { scale: 1.02, opacity: 1 } : { scale: 1, opacity: 0 }}
        transition={{ duration: 0.3 }}
      />
      
      {/* Subtle glow border */}
      <motion.div 
        className="absolute inset-0 bg-gradient-to-r from-primary/20 via-transparent to-primary/20 rounded-3xl blur-sm opacity-0 group-hover:opacity-100 transition-all duration-700"
        animate={isHovered ? { scale: 1.05 } : { scale: 1 }}
        transition={{ duration: 0.5 }}
      />

      {/* Icon with enhanced animations */}
      <div className="relative z-10 flex justify-center mb-8">
        <motion.div 
          className="relative"
          whileHover={{ rotate: [0, -5, 5, 0] }}
          transition={{ duration: 0.6, ease: "easeInOut" }}
        >
          <motion.div 
            className="absolute inset-0 bg-primary/20 rounded-2xl blur-xl"
            animate={isHovered ? 
              { scale: 1.4, opacity: 0.8, rotate: 180 } : 
              { scale: 1, opacity: 0.3, rotate: 0 }
            }
            transition={{ duration: 0.8, ease: "easeOut" }}
          />
          <motion.div 
            className="relative bg-gradient-to-br from-primary via-primary/90 to-primary/70 p-5 rounded-2xl shadow-xl"
            whileHover={{ scale: 1.1 }}
            transition={{ duration: 0.3, ease: "easeOut" }}
          >
            <motion.div
              animate={isHovered ? { rotate: 360 } : { rotate: 0 }}
              transition={{ duration: 1.2, ease: "easeInOut" }}
            >
              <Icon className="h-8 w-8 text-white" />
            </motion.div>
          </motion.div>
        </motion.div>
      </div>

      {/* Title with smooth animation */}
      <motion.h3 
        className="text-xl font-semibold text-center mb-4 transition-colors duration-300"
        whileHover={{ scale: 1.05 }}
        transition={{ duration: 0.2 }}
      >
        {title}
      </motion.h3>

      {/* Description */}
      <motion.p 
        className="text-muted-foreground text-center text-sm leading-relaxed mb-6 flex-grow"
        initial={{ opacity: 0.8 }}
        whileHover={{ opacity: 1 }}
        transition={{ duration: 0.3 }}
      >
        {description}
      </motion.p>

      {/* Technologies with stagger animation */}
      <motion.div 
        className="overflow-hidden"
        initial={{ height: 0, opacity: 0 }}
        animate={isHovered ? 
          { height: "auto", opacity: 1 } : 
          { height: 0, opacity: 0 }
        }
        transition={{ 
          duration: 0.5, 
          ease: [0.25, 0.46, 0.45, 0.94],
          staggerChildren: 0.1
        }}
      >
        <div className="border-t border-border/50 pt-6">
          <motion.p 
            className="text-xs font-semibold text-primary text-center mb-4"
            initial={{ y: 10, opacity: 0 }}
            animate={isHovered ? { y: 0, opacity: 1 } : { y: 10, opacity: 0 }}
            transition={{ duration: 0.3, delay: 0.1 }}
          >
            Core Technologies
          </motion.p>
          <div className="flex flex-wrap justify-center gap-2">
            {technologies.map((tech, techIndex) => (
              <motion.span
                key={techIndex}
                className="inline-block px-3 py-1.5 text-xs bg-secondary/80 text-secondary-foreground rounded-full border border-secondary/30 backdrop-blur-sm hover:bg-primary/10 hover:border-primary/50 transition-all duration-300"
                initial={{ scale: 0, opacity: 0 }}
                animate={isHovered ? 
                  { scale: 1, opacity: 1 } : 
                  { scale: 0, opacity: 0 }
                }
                transition={{ 
                  duration: 0.3, 
                  delay: techIndex * 0.05,
                  ease: "easeOut"
                }}
                whileHover={{ scale: 1.1, y: -2 }}
              >
                {tech}
              </motion.span>
            ))}
          </div>
        </div>
      </motion.div>

      {/* CTA Button with smooth reveal */}
      <motion.div 
        className="flex justify-center mt-6"
        initial={{ y: 20, opacity: 0 }}
        animate={isHovered ? { y: 0, opacity: 1 } : { y: 20, opacity: 0 }}
        transition={{ duration: 0.4, delay: 0.2 }}
      >
        <motion.div
          whileHover={{ scale: 1.05 }}
          whileTap={{ scale: 0.95 }}
        >
          <Link href={link}>
            <Button 
              variant="outline" 
              size="sm"
              className="text-xs bg-background/50 backdrop-blur-sm border-primary/30 hover:bg-primary hover:text-primary-foreground hover:shadow-lg transition-all duration-300"
            >
              Learn More
            </Button>
          </Link>
        </motion.div>
      </motion.div>
    </motion.div>
  );
};


export default function ServicesSection() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const services = [
    {
      icon: Bot,
      title: "AI-Powered Solutions",
      description: "Custom ML models, LLMs, real-time pipelines, and computer vision systems.",
      technologies: ["PyTorch", "TensorFlow", "OpenCV", "Transformers"],
      link: "/services/ai-solutions",
    },
    {
      icon: Zap,
      title: "Embedded Systems & IoT",
      description: "Smart sensors, LoRa, ESP32 systems, and real-time wireless solutions.",
      technologies: ["ESP32", "LoRa", "MQTT", "Arduino"],
      link: "/services/embedded-iot",
    },
    {
      icon: Layers,
      title: "Robotics & Autonomous Systems",
      description: "SLAM, path planning, ROS2-based robots for automation and navigation.",
      technologies: ["ROS2", "SLAM", "OpenCV", "Gazebo"],
      link: "/services/robotics",
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
      link: "/services/cloud-development",
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
      link: "/services/devops",
    },
    {
      icon: FlaskRound,
      title: "AI/ML Research & Consulting",
      description: "Model fine-tuning, system design, research support, ethical AI strategies.",
      technologies: ["Research", "Consulting", "Ethics", "Strategy"],
      link: "/services/ai-research",
    },
  ];

  return (
    <div className="min-h-screen bg-background">
      <section ref={ref} id="services" className="py-24 bg-gradient-to-br from-background via-muted/5 to-background relative overflow-hidden">
        
        {/* Enhanced background effects */}
        <div className="absolute inset-0">
          <motion.div 
            className="absolute top-1/4 left-1/3 w-96 h-96 bg-primary/5 rounded-full blur-3xl"
            animate={{ 
              scale: [1, 1.2, 1],
              opacity: [0.3, 0.5, 0.3],
            }}
            transition={{ 
              duration: 8,
              repeat: Infinity,
              ease: "easeInOut"
            }}
          />
          <motion.div 
            className="absolute bottom-1/4 right-1/3 w-80 h-80 bg-secondary/5 rounded-full blur-3xl"
            animate={{ 
              scale: [1.2, 1, 1.2],
              opacity: [0.2, 0.4, 0.2],
            }}
            transition={{ 
              duration: 10,
              repeat: Infinity,
              ease: "easeInOut",
              delay: 2
            }}
          />
          <motion.div 
            className="absolute top-1/2 left-1/2 w-64 h-64 bg-primary/3 rounded-full blur-2xl transform -translate-x-1/2 -translate-y-1/2"
            animate={{ 
              rotate: [0, 360],
              scale: [1, 1.3, 1],
            }}
            transition={{ 
              duration: 15,
              repeat: Infinity,
              ease: "linear"
            }}
          />
        </div>

        <div className="container mx-auto px-4 relative z-10">
          {/* Header with stagger animation */}
          <motion.div 
            className="text-center mb-20"
            initial={{ opacity: 0, y: 30 }}
            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
            transition={{ duration: 0.8, ease: [0.25, 0.46, 0.45, 0.94] }}
          >
            <motion.h2 
              className="text-5xl md:text-6xl font-bold tracking-tight mb-6 bg-gradient-to-r from-foreground via-foreground to-foreground/80 bg-clip-text text-transparent"
              initial={{ opacity: 0, y: 20 }}
              animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 20 }}
              transition={{ duration: 0.8, delay: 0.2 }}
            >
              Our Services
            </motion.h2>
            <motion.p 
              className="text-muted-foreground max-w-3xl mx-auto text-lg leading-relaxed"
              initial={{ opacity: 0, y: 20 }}
              animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 20 }}
              transition={{ duration: 0.8, delay: 0.4 }}
            >
              Explore a comprehensive suite of cutting-edge solutions meticulously designed to drive innovation and accelerate digital transformation across industries.
            </motion.p>
          </motion.div>

          {/* Services Grid with enhanced container */}
          <motion.div 
            className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 max-w-7xl mx-auto"
            variants={{
              hidden: { opacity: 0 },
              show: {
                opacity: 1,
                transition: {
                  staggerChildren: 0.15,
                  delayChildren: 0.3,
                }
              }
            }}
            initial="hidden"
            animate={isInView ? "show" : "hidden"}
          >
            {services.map((service, index) => (
              <motion.div
                key={index}
                variants={{
                  hidden: { opacity: 0, y: 50, scale: 0.9 },
                  show: { 
                    opacity: 1, 
                    y: 0, 
                    scale: 1,
                    transition: {
                      duration: 0.8,
                      ease: [0.25, 0.46, 0.45, 0.94]
                    }
                  }
                }}
              >
                <ServiceCard {...service} index={index} />
              </motion.div>
            ))}
          </motion.div>

          {/* Enhanced CTA Button */}
          <motion.div 
            className="text-center mt-20"
            initial={{ opacity: 0, y: 30 }}
            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
            transition={{ duration: 0.8, delay: 1.2 }}
          >
            <motion.a
              href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27d%20like%20to%20connect%20with%20the%20Aura%20Digital%20Labs%20team%20for%20a%20discussion."
              target="_blank"
              rel="noopener noreferrer"
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              transition={{ duration: 0.2 }}
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-primary via-primary/90 to-primary/80 hover:from-primary/90 hover:to-primary text-primary-foreground font-semibold px-10 py-4 rounded-full tracking-wide shadow-xl hover:shadow-2xl transition-all duration-300 backdrop-blur-sm border border-primary/20"
              >
                Book a Free Consultation
              </Button>
            </motion.a>
          </motion.div>
        </div>
      </section>
    </div>
  );
}
