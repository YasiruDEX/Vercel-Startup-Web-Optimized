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
    "ROS2", "SLAM", "OpenCV", "Gazebo", "MoveIt", "PCL", 
    "TensorFlow", "PyTorch", "Lidar", "IMU", "Kalman Filters", "PID Control"
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

      <FooterSection />
    </div>
  );
}
