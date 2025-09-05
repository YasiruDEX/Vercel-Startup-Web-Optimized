"use client";

import { motion, useInView } from "framer-motion";
import { useRef } from "react";
import { Button } from "@/components/ui/button";
import { Layers, Navigation, Camera, MapPin, Cog, Bot, ArrowLeft, CheckCircle } from "lucide-react";
import Link from "next/link";
import HeaderSection from "@/components/header";
import FooterSection from "@/components/footerSection";

export default function RoboticsPage() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const features = [
    {
      icon: Navigation,
      title: "SLAM Technology",
      description: "Simultaneous Localization and Mapping for autonomous navigation in complex environments."
    },
    {
      icon: MapPin,
      title: "Path Planning",
      description: "Advanced algorithms for optimal route planning and obstacle avoidance in real-time."
    },
    {
      icon: Camera,
      title: "Computer Vision",
      description: "Real-time object detection, recognition, and tracking for intelligent automation."
    },
    {
      icon: Cog,
      title: "ROS2 Integration",
      description: "Robot Operating System 2.0 for scalable and modular robotic system development."
    }
  ];

  const technologies = [
    "ROS2", "OpenCV", "TensorFlow", "PyTorch", "Gazebo",
    "MoveIt", "Lidar", "IMU", "SLAM", "Robot Operating System"
  ];

  const applications = [
    {
      title: "Autonomous Navigation",
      description: "Self-driving robots for warehouses, hospitals, and industrial environments.",
      benefits: ["24/7 operation", "Precision navigation", "Safety protocols"]
    },
    {
      title: "Manufacturing Automation",
      description: "Robotic arms and automated systems for assembly and quality control.",
      benefits: ["Increased productivity", "Quality consistency", "Cost reduction"]
    },
    {
      title: "Service Robots",
      description: "Interactive robots for customer service, delivery, and assistance applications.",
      benefits: ["Human-robot interaction", "Task automation", "Scalable deployment"]
    }
  ];
  const useCases = [
    {
      title: "Autonomous Vehicles",
      description: "Self-navigating ground and aerial robots for logistics and inspection.",
      benefits: ["Reduced labor", "24/7 operations", "Route optimization"]
    },
    {
      title: "Precision Manufacturing",
      description: "Robotic arms for assembly, welding, and quality assurance with high accuracy.",
      benefits: ["Consistent production", "Lower error rates", "Faster cycle times"]
    },
    {
      title: "Service & Assistance",
      description: "Interactive robots for customer service, healthcare support, and facility management.",
      benefits: ["Improved engagement", "Scalable deployment", "24/7 availability"]
    }
  ];

  return (
    <div className="min-h-screen bg-background">
      <HeaderSection />
      
      <section className="pt-24 pb-16 bg-gradient-to-br from-purple-500/5 via-background to-blue-500/5 relative overflow-hidden">
        <div className="container mx-auto px-4 relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            className="max-w-4xl mx-auto text-center"
          >
            <Link href="/" className="inline-flex items-center gap-2 text-primary hover:text-primary/80 mb-8 transition-colors">
              <ArrowLeft className="w-4 h-4" />
              Back to Home
            </Link>
            
            <div className="flex justify-center mb-8">
              <motion.div 
                className="bg-gradient-to-br from-purple-500 via-purple-600 to-blue-500 p-6 rounded-3xl shadow-xl"
                whileHover={{ scale: 1.05, rotate: 5 }}
                transition={{ duration: 0.3 }}
              >
                <Layers className="h-12 w-12 text-white" />
              </motion.div>
            </div>

            <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-6 bg-gradient-to-r from-foreground via-foreground to-foreground/80 bg-clip-text text-transparent">
              Robotics & Autonomous Systems
            </h1>
            <p className="text-xl text-muted-foreground leading-relaxed mb-8 max-w-3xl mx-auto">
              Build the future with advanced robotics and autonomous systems. From SLAM navigation to intelligent automation, 
              we create robots that work seamlessly in real-world environments.
            </p>
            
            <motion.div 
              className="flex flex-col sm:flex-row gap-4 justify-center"
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-purple-500 via-purple-600 to-blue-500 hover:from-purple-600 hover:to-blue-600 text-white font-semibold px-8 py-4 rounded-full"
              >
                <a 
                  href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Robotics%20%26%20Autonomous%20Systems.%20Let%27s%20discuss!"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="flex items-center gap-2"
                >
                  Start Your Project
                  <Bot className="w-4 h-4" />
                </a>
              </Button>
            </motion.div>
          </motion.div>
        </div>
      </section>

      <section ref={ref} id="features" className="py-24 bg-background">
        <div className="container mx-auto px-4">
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
            {features.map((feature, index) => {
              const IconComponent = feature.icon;
              return (
                <motion.div
                  key={index}
                  initial={{ opacity: 0, y: 50 }}
                  animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 50 }}
                  transition={{ duration: 0.8, delay: index * 0.1 }}
                  className="group bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-6 hover:border-purple-500/30 hover:bg-card/80 transition-all duration-300"
                >
                  <div className="flex justify-center mb-4">
                    <div className="bg-gradient-to-br from-purple-500 via-purple-600 to-blue-500 p-3 rounded-xl">
                      <IconComponent className="h-6 w-6 text-white" />
                    </div>
                  </div>
                  <h3 className="text-xl font-semibold text-center mb-3">{feature.title}</h3>
                  <p className="text-muted-foreground text-center text-sm leading-relaxed">
                    {feature.description}
                  </p>
                </motion.div>
              );
            })}
          </div>
        </div>
      </section>
      {/* Technologies Section */}
      <section className="py-24 bg-muted/30">
        <div className="container mx-auto px-4">
          <motion.div className="text-center mb-16" initial={{ opacity: 0, y: 30 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }} viewport={{ once: true }}>
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Technologies We Leverage</h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">Core tools and frameworks powering our robotics solutions.</p>
          </motion.div>
          <motion.div className="flex flex-wrap justify-center gap-4 max-w-4xl mx-auto" initial={{ opacity: 0 }} whileInView={{ opacity: 1 }} transition={{ duration: 0.8, staggerChildren: 0.1 }} viewport={{ once: true }}>
            {technologies.map((tech, idx) => (
              <motion.span key={idx} initial={{ scale: 0, opacity: 0 }} whileInView={{ scale: 1, opacity: 1 }} transition={{ duration: 0.5, delay: idx * 0.05 }} viewport={{ once: true }} className="inline-block px-4 py-2 bg-background border border-border/50 rounded-full text-sm font-medium hover:border-purple-500/50 hover:bg-purple-500/5 transition-all duration-300">
                {tech}
              </motion.span>
            ))}
          </motion.div>
        </div>
      </section>
      {/* Use Cases Section */}
      <section className="py-24 bg-background">
        <div className="container mx-auto px-4">
          <motion.div className="text-center mb-16" initial={{ opacity: 0, y: 30 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }} viewport={{ once: true }}>
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Industry Use Cases</h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">Practical applications demonstrating our robotics expertise.</p>
          </motion.div>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {useCases.map((uc, i) => (
              <motion.div key={i} initial={{ opacity: 0, y: 50 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8, delay: i * 0.2 }} viewport={{ once: true }} className="bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-8 hover:border-purple-500/30 hover:bg-card/80 transition-all duration-300">
                <h3 className="text-2xl font-bold mb-4">{uc.title}</h3>
                <p className="text-muted-foreground mb-6">{uc.description}</p>
                <div className="space-y-3">
                  {uc.benefits.map((b, bi) => (
                    <div key={bi} className="flex items-center gap-3"><CheckCircle className="w-5 h-5 text-purple-500 flex-shrink-0" /><span className="text-sm">{b}</span></div>
                  ))}
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>
      {/* CTA Section */}
      <section className="py-24 bg-gradient-to-br from-purple-500/5 via-background to-blue-500/5">
        <div className="container mx-auto px-4 text-center">
          <motion.div initial={{ opacity: 0, y: 30 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }} viewport={{ once: true }} className="max-w-3xl mx-auto">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Ready to Innovate with Robotics?</h2>
            <p className="text-xl text-muted-foreground mb-8">Let’s build autonomous systems that transform industries.</p>
            <Button size="lg" className="bg-gradient-to-r from-purple-500 via-purple-600 to-blue-500 hover:from-purple-600 hover:to-blue-600 text-white font-semibold px-10 py-4 rounded-full">
              <a href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Robotics%20%26%20Autonomous%20Systems.%20Let%27s%20discuss!" target="_blank" rel="noopener noreferrer" className="flex items-center gap-2">
                Start Your Robotics Project<Bot className="w-4 h-4" />
              </a>
            </Button>
          </motion.div>
        </div>
      </section>
      <FooterSection />
    </div>
  );
}
