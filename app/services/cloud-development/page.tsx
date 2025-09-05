"use client";

import { motion, useInView } from "framer-motion";
import { useRef } from "react";
import { Button } from "@/components/ui/button";
import { Cloud, Globe, Smartphone, Database, Server, Code, ArrowLeft, CheckCircle } from "lucide-react";
import Link from "next/link";
import HeaderSection from "@/components/header";
import FooterSection from "@/components/footerSection";

export default function CloudDevelopmentPage() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const features = [
    {
      icon: Globe,
      title: "Web Applications",
      description: "Modern, responsive web applications built with cutting-edge frameworks and technologies."
    },
    {
      icon: Smartphone,
      title: "Mobile Development",
      description: "Cross-platform mobile apps for iOS and Android using React Native and Flutter."
    },
    {
      icon: Database,
      title: "Database Design",
      description: "Scalable database architectures with SQL and NoSQL solutions for optimal performance."
    },
    {
      icon: Server,
      title: "Cloud Integration",
      description: "Seamless cloud deployment and integration with AWS, GCP, and Azure platforms."
    }
  ];

  const technologies = [
    "Node.js", "React", "Flutter", "Next.js", "Express.js", "MongoDB", 
    "MySQL", "React Native", "Flask", "Spring Boot", "AWS", "Docker"
  ];

  const useCases = [
    {
      title: "E-commerce Platforms",
      description: "Scalable, secure online stores with payment integrations and user-friendly interfaces.",
      benefits: ["Custom storefronts", "Seamless checkout", "High-availability"]
    },
    {
      title: "Enterprise Portals",
      description: "Robust internal portals for collaboration, reporting, and dashboards.",
      benefits: ["Secure access", "Real-time data", "Custom workflows"]
    },
    {
      title: "API Services",
      description: "RESTful and GraphQL APIs for mobile and web applications.",
      benefits: ["High performance", "Scalable endpoints", "Comprehensive docs"]
    }
  ];

  return (
    <div className="min-h-screen bg-background">
      <HeaderSection />
      
      <section className="pt-24 pb-16 bg-gradient-to-br from-blue-500/5 via-background to-cyan-500/5 relative overflow-hidden">
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
                className="bg-gradient-to-br from-blue-500 via-blue-600 to-cyan-500 p-6 rounded-3xl shadow-xl"
                whileHover={{ scale: 1.05, rotate: 5 }}
                transition={{ duration: 0.3 }}
              >
                <Cloud className="h-12 w-12 text-white" />
              </motion.div>
            </div>

            <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-6 bg-gradient-to-r from-foreground via-foreground to-foreground/80 bg-clip-text text-transparent">
              Cloud & App Development
            </h1>
            <p className="text-xl text-muted-foreground leading-relaxed mb-8 max-w-3xl mx-auto">
              Build scalable web and mobile applications with modern cloud infrastructure. 
              From responsive websites to cross-platform mobile apps, we deliver complete digital solutions.
            </p>
            
            <motion.div 
              className="flex flex-col sm:flex-row gap-4 justify-center"
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-blue-500 via-blue-600 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white font-semibold px-8 py-4 rounded-full"
              >
                <a 
                  href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Cloud%20%26%20App%20Development.%20Let%27s%20discuss!"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="flex items-center gap-2"
                >
                  Start Your Project
                  <Code className="w-4 h-4" />
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
                  className="group bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-6 hover:border-blue-500/30 hover:bg-card/80 transition-all duration-300"
                >
                  <div className="flex justify-center mb-4">
                    <div className="bg-gradient-to-br from-blue-500 via-blue-600 to-cyan-500 p-3 rounded-xl">
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
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Technologies We Use</h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">Modern frameworks and cloud services powering your applications.</p>
          </motion.div>
          <motion.div className="flex flex-wrap justify-center gap-4 max-w-4xl mx-auto" initial={{ opacity: 0 }} whileInView={{ opacity: 1 }} transition={{ duration: 0.8, staggerChildren: 0.1 }} viewport={{ once: true }}>
            {technologies.map((tech, idx) => (
              <motion.span key={idx} initial={{ scale: 0, opacity: 0 }} whileInView={{ scale: 1, opacity: 1 }} transition={{ duration: 0.5, delay: idx * 0.05 }} viewport={{ once: true }} className="inline-block px-4 py-2 bg-background border border-border/50 rounded-full text-sm font-medium hover:border-blue-500/50 hover:bg-blue-500/5 transition-all duration-300">
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
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Application Examples</h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">Real-world implementations of our cloud and app development services.</p>
          </motion.div>
          <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {useCases.map((uc, i) => (
              <motion.div key={i} initial={{ opacity: 0, y: 50 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8, delay: i * 0.2 }} viewport={{ once: true }} className="bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-8 hover:border-blue-500/30 hover:bg-card/80 transition-all duration-300">
                <h3 className="text-2xl font-bold mb-4">{uc.title}</h3>
                <p className="text-muted-foreground mb-6">{uc.description}</p>
                <div className="space-y-3">
                  {uc.benefits.map((b, bi) => (
                    <div key={bi} className="flex items-center gap-3"><CheckCircle className="w-5 h-5 text-blue-500 flex-shrink-0" /><span className="text-sm">{b}</span></div>
                  ))}
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>
      {/* CTA Section */}
      <section className="py-24 bg-gradient-to-br from-blue-500/5 via-background to-cyan-500/5">
        <div className="container mx-auto px-4 text-center">
          <motion.div initial={{ opacity: 0, y: 30 }} whileInView={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }} viewport={{ once: true }} className="max-w-3xl mx-auto">
            <h2 className="text-4xl md:text-5xl font-bold mb-6">Ready to Build Your Next App?</h2>
            <p className="text-xl text-muted-foreground mb-8">Let's create scalable web and mobile solutions together.</p>
            <Button size="lg" className="bg-gradient-to-r from-blue-500 via-blue-600 to-cyan-500 hover:from-blue-600 hover:to-cyan-600 text-white font-semibold px-10 py-4 rounded-full">
              <a href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20Cloud%20%26%20App%20Development.%20Let%27s%20discuss!" target="_blank" rel="noopener noreferrer" className="flex items-center gap-2">
                Start Your Project<Code className="w-4 h-4" />
              </a>
            </Button>
          </motion.div>
        </div>
      </section>
      <FooterSection />
    </div>
  );
}
