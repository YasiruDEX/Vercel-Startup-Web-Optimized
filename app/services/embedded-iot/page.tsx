"use client";

import { motion, useInView } from "framer-motion";
import { useRef } from "react";
import { Button } from "@/components/ui/button";
import { Zap, Wifi, Cpu, Radio, Shield, Battery, ArrowLeft, CheckCircle } from "lucide-react";
import Link from "next/link";
import HeaderSection from "@/components/header";
import FooterSection from "@/components/footerSection";

export default function EmbeddedIoTPage() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const features = [
    {
      icon: Cpu,
      title: "Smart Sensors",
      description: "Advanced sensor integration with real-time data processing and edge computing capabilities."
    },
    {
      icon: Radio,
      title: "LoRa Networks",
      description: "Long-range, low-power wireless communication for large-scale IoT deployments."
    },
    {
      icon: Wifi,
      title: "ESP32 Systems",
      description: "Powerful microcontroller solutions with built-in Wi-Fi and Bluetooth connectivity."
    },
    {
      icon: Shield,
      title: "Secure Protocols",
      description: "End-to-end encryption and secure communication protocols for IoT device protection."
    }
  ];

  const technologies = [
    "ESP32", "LoRa", "MQTT", "Zigbee", "Thread",
    "FreeRTOS", "Docker", "AWS IoT Core", "Azure IoT Hub", "Edge Computing"
  ];

  const solutions = [
    {
      title: "Smart Agriculture",
      description: "IoT sensors for soil monitoring, irrigation control, and crop management systems.",
      benefits: ["30% water savings", "Increased crop yield", "Remote monitoring"],
      image: "/services/agriculture-iot.png"
    },
    {
      title: "Industrial Automation",
      description: "Connected factory solutions with predictive maintenance and real-time monitoring.",
      benefits: ["Reduced downtime", "Predictive analytics", "Energy optimization"],
      image: "/services/industrial-iot.png"
    },
    {
      title: "Smart Buildings",
      description: "Building automation systems for HVAC, lighting, security, and energy management.",
      benefits: ["40% energy reduction", "Enhanced security", "Comfort optimization"],
      image: "/services/smart-building.png"
    }
  ];
  const useCases = solutions; // reuse solutions array as useCases

  return (
    <div className="min-h-screen bg-background">
      <HeaderSection />
      
      {/* Hero Section */}
      <section className="pt-24 pb-16 bg-gradient-to-br from-orange-500/5 via-background to-yellow-500/5 relative overflow-hidden">
        <div className="absolute inset-0">
          <motion.div 
            className="absolute top-1/4 right-1/4 w-96 h-96 bg-orange-500/10 rounded-full blur-3xl"
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
        </div>

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
                className="bg-gradient-to-br from-orange-500 via-orange-600 to-yellow-500 p-6 rounded-3xl shadow-xl"
                whileHover={{ scale: 1.05, rotate: 5 }}
                transition={{ duration: 0.3 }}
              >
                <Zap className="h-12 w-12 text-white" />
              </motion.div>
            </div>

            <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-6 bg-gradient-to-r from-foreground via-foreground to-foreground/80 bg-clip-text text-transparent">
              Embedded Systems & IoT
            </h1>
            <p className="text-xl text-muted-foreground leading-relaxed mb-8 max-w-3xl mx-auto">
              Connect the physical and digital worlds with our cutting-edge embedded systems and IoT solutions. 
              From smart sensors to wireless networks, we create intelligent systems that transform how you interact with technology.
            </p>
            
            <motion.div 
              className="flex flex-col sm:flex-row gap-4 justify-center"
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-orange-500 via-orange-600 to-yellow-500 hover:from-orange-600 hover:to-yellow-600 text-white font-semibold px-8 py-4 rounded-full"
              >
                <a 
                  href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Embedded%20Systems%20%26%20IoT%20Solutions.%20Let%27s%20discuss!"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="flex items-center gap-2"
                >
                  Get Started
                  <Wifi className="w-4 h-4" />
                </a>
              </Button>
              <Button 
                variant="outline" 
                size="lg"
                className="border-orange-500/30 hover:bg-orange-500/10 px-8 py-4 rounded-full"
              >
                <a 
                  href="#features"
                  className="flex items-center gap-2"
                >
                  Explore Solutions
                  <ArrowLeft className="w-4 h-4 rotate-270" />
                </a>
              </Button>
            </motion.div>
          </motion.div>
        </div>
      </section>

      {/* Features Section */}
      <section ref={ref} id="features" className="py-24 bg-background">
        <div className="container mx-auto px-4">
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 30 }}
            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Advanced IoT Capabilities
            </h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
              Comprehensive embedded systems and IoT solutions for next-generation connectivity.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
            {features.map((feature, index) => {
              const IconComponent = feature.icon;
              return (
                <motion.div
                  key={index}
                  initial={{ opacity: 0, y: 50 }}
                  animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 50 }}
                  transition={{ duration: 0.8, delay: index * 0.1 }}
                  className="group bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-6 hover:border-orange-500/30 hover:bg-card/80 transition-all duration-300"
                >
                  <div className="flex justify-center mb-4">
                    <div className="bg-gradient-to-br from-orange-500 via-orange-600 to-yellow-500 p-3 rounded-xl">
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
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Protocols & Tools</h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">Key hardware platforms and communication protocols we implement.</p>
          </motion.div>
          <motion.div className="flex flex-wrap justify-center gap-4 max-w-4xl mx-auto" initial={{ opacity: 0 }} whileInView={{ opacity: 1 }} transition={{ duration: 0.8, staggerChildren: 0.1 }} viewport={{ once: true }}>
            {technologies.map((tech, idx) => (
              <motion.span key={idx} initial={{ scale: 0, opacity: 0 }} whileInView={{ scale: 1, opacity: 1 }} transition={{ duration: 0.5, delay: idx * 0.05 }} viewport={{ once: true }} className="inline-block px-4 py-2 bg-background border border-border/50 rounded-full text-sm font-medium hover:border-orange-500/50 hover:bg-orange-500/5 transition-all duration-300">
                {tech}
              </motion.span>
            ))}
          </motion.div>
        </div>
      </section>

      {/* Solutions Section */}
      <section className="py-24 bg-background">
        <div className="container mx-auto px-4">
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Industry Solutions
            </h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
              Transforming industries with intelligent embedded systems and IoT connectivity.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {solutions.map((solution, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 50 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.8, delay: index * 0.2 }}
                viewport={{ once: true }}
                className="bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-8 hover:border-orange-500/30 hover:bg-card/80 transition-all duration-300"
              >
                <h3 className="text-2xl font-bold mb-4">{solution.title}</h3>
                <p className="text-muted-foreground mb-6">{solution.description}</p>
                <div className="space-y-3">
                  {solution.benefits.map((benefit, idx) => (
                    <div key={idx} className="flex items-center gap-3">
                      <CheckCircle className="w-5 h-5 text-orange-500 flex-shrink-0" />
                      <span className="text-sm">{benefit}</span>
                    </div>
                  ))}
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Performance Stats */}
      <section className="py-24 bg-gradient-to-br from-orange-500/5 via-background to-yellow-500/5">
        <div className="container mx-auto px-4">
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Proven Performance
            </h2>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-4 gap-8 max-w-4xl mx-auto">
            {[
              { metric: "99.9%", label: "Uptime Reliability" },
              { metric: "10km+", label: "LoRa Range" },
              { metric: "5 Years", label: "Battery Life" },
              { metric: "1000+", label: "Devices Deployed" }
            ].map((stat, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, scale: 0.5 }}
                whileInView={{ opacity: 1, scale: 1 }}
                transition={{ duration: 0.8, delay: index * 0.1 }}
                viewport={{ once: true }}
                className="text-center"
              >
                <div className="text-4xl md:text-5xl font-bold text-orange-500 mb-2">
                  {stat.metric}
                </div>
                <div className="text-muted-foreground">{stat.label}</div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-24 bg-gradient-to-br from-orange-500/5 via-background to-yellow-500/5">
        <div className="container mx-auto px-4 text-center">
          <motion.div initial={{ opacity: 0, y: 30 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }} viewport={{ once: true }} className="max-w-3xl mx-auto">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Ready for Smart IoT Solutions?</h2>
            <p className="text-xl text-muted-foreground mb-8">Let&apos;s build connected systems that drive efficiency and insights.</p>
            <Button size="lg" className="bg-gradient-to-r from-orange-500 via-orange-600 to-yellow-500 hover:from-orange-600 hover:to-yellow-600 text-white font-semibold px-10 py-4 rounded-full">
              <a href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Embedded%20Systems%20%26%20IoT.%20Let%27s%20discuss!" target="_blank" rel="noopener noreferrer" className="flex items-center gap-2">
                Start Your IoT Project
                <Wifi className="w-4 h-4" />
              </a>
            </Button>
          </motion.div>
        </div>
      </section>

      <FooterSection />
    </div>
  );
}
