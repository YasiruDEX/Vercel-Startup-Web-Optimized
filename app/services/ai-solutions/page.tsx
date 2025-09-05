"use client";

import { motion, useInView } from "framer-motion";
import { useRef } from "react";
import { Button } from "@/components/ui/button";
import { Bot, Brain, Eye, MessageSquare, TrendingUp, Zap, ArrowLeft, CheckCircle } from "lucide-react";
import Link from "next/link";
import HeaderSection from "@/components/header";
import FooterSection from "@/components/footerSection";

export default function AISolutionsPage() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const features = [
    {
      icon: Brain,
      title: "Custom ML Models",
      description: "Tailored machine learning models designed specifically for your business needs and data patterns."
    },
    {
      icon: MessageSquare,
      title: "LLM Integration",
      description: "Large Language Model integration for chatbots, content generation, and intelligent automation."
    },
    {
      icon: Eye,
      title: "Computer Vision",
      description: "Advanced image and video analysis for quality control, surveillance, and automation systems."
    },
    {
      icon: TrendingUp,
      title: "Real-time Analytics",
      description: "Live data processing and insights generation for immediate decision-making capabilities."
    }
  ];

  const technologies = [
    "PyTorch", "TensorFlow", "OpenCV", "Transformers", "Scikit-learn", 
    "CUDA", "Docker", "Kubernetes", "AWS SageMaker", "Google AI Platform"
  ];

  const useCases = [
    {
      title: "Healthcare AI",
      description: "Medical image analysis, diagnostic assistance, and patient data processing systems.",
      benefits: ["95% accuracy in diagnostics", "50% faster processing", "24/7 availability"]
    },
    {
      title: "Financial Services",
      description: "Fraud detection, risk assessment, and algorithmic trading solutions.",
      benefits: ["Real-time fraud detection", "Risk reduction", "Automated compliance"]
    },
    {
      title: "Manufacturing",
      description: "Quality control, predictive maintenance, and supply chain optimization.",
      benefits: ["30% quality improvement", "Reduced downtime", "Cost optimization"]
    }
  ];

  return (
    <div className="min-h-screen bg-background">
      <HeaderSection />
      
      {/* Hero Section */}
  <section className="pt-24 pb-16 bg-gradient-to-br from-primary/5 to-secondary/5 relative overflow-hidden">
        <div className="absolute inset-0">
          {/* <motion.div 
            className="absolute top-1/4 left-1/4 w-96 h-96 bg-primary/10 rounded-full blur-3xl"
            animate={{ 
              scale: [1, 1.2, 1],
              opacity: [0.3, 0.5, 0.3],
            }}
            transition={{ 
              duration: 8,
              repeat: Infinity,
              ease: "easeInOut"
            }}
          /> */}
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
                className="bg-gradient-to-br from-primary via-primary/90 to-primary/70 p-6 rounded-3xl shadow-xl"
                whileHover={{ scale: 1.05, rotate: 5 }}
                transition={{ duration: 0.3 }}
              >
                <Bot className="h-12 w-12 text-white" />
              </motion.div>
            </div>

            <h1 className="text-5xl md:text-7xl font-bold tracking-tight mb-6 bg-gradient-to-r from-foreground via-foreground to-foreground/80 bg-clip-text text-transparent">
              AI-Powered Solutions
            </h1>
            <p className="text-xl text-muted-foreground leading-relaxed mb-8 max-w-3xl mx-auto">
              Transform your business with cutting-edge artificial intelligence solutions. 
              From custom machine learning models to real-time analytics, we deliver AI systems 
              that drive innovation and competitive advantage.
            </p>
            
            <motion.div 
              className="flex flex-col sm:flex-row gap-4 justify-center"
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
            >
              <Button 
                size="lg"
                className="bg-gradient-to-r from-primary via-primary/90 to-primary/80 hover:from-primary/90 hover:to-primary text-primary-foreground font-semibold px-8 py-4 rounded-full"
              >
                <a 
                  href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20AI-Powered%20Solutions.%20Let%27s%20discuss!"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="flex items-center gap-2"
                >
                  Get Started
                  <Zap className="w-4 h-4" />
                </a>
              </Button>
              <Button 
                variant="outline" 
                size="lg"
                className="border-primary/30 hover:bg-primary/10 px-8 py-4 rounded-full"
              >
                <a 
                  href="#features"
                  className="flex items-center gap-2"
                >
                  Learn More
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
              Advanced AI Capabilities
            </h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
              Our comprehensive AI solutions cover the full spectrum of artificial intelligence technologies.
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
                  className="group bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-6 hover:border-primary/30 hover:bg-card/80 transition-all duration-300"
                >
                  <div className="flex justify-center mb-4">
                    <div className="bg-gradient-to-br from-primary via-primary/90 to-primary/70 p-3 rounded-xl">
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
          <motion.div 
            className="text-center mb-16"
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Technologies We Use
            </h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
              Industry-leading tools and frameworks for robust AI development.
            </p>
          </motion.div>

          <motion.div 
            className="flex flex-wrap justify-center gap-4 max-w-4xl mx-auto"
            initial={{ opacity: 0 }}
            whileInView={{ opacity: 1 }}
            transition={{ duration: 0.8, staggerChildren: 0.1 }}
            viewport={{ once: true }}
          >
            {technologies.map((tech, index) => (
              <motion.span
                key={index}
                initial={{ scale: 0, opacity: 0 }}
                whileInView={{ scale: 1, opacity: 1 }}
                transition={{ duration: 0.5, delay: index * 0.05 }}
                viewport={{ once: true }}
                className="inline-block px-4 py-2 bg-background border border-border/50 rounded-full text-sm font-medium hover:border-primary/50 hover:bg-primary/5 transition-all duration-300"
              >
                {tech}
              </motion.span>
            ))}
          </motion.div>
        </div>
      </section>

      {/* Use Cases Section */}
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
              Industry Applications
            </h2>
            <p className="text-xl text-muted-foreground max-w-3xl mx-auto">
              Real-world AI solutions delivering measurable results across industries.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8 max-w-6xl mx-auto">
            {useCases.map((useCase, index) => (
              <motion.div
                key={index}
                initial={{ opacity: 0, y: 50 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.8, delay: index * 0.2 }}
                viewport={{ once: true }}
                className="bg-card/50 backdrop-blur-sm rounded-2xl border border-border/50 p-8 hover:border-primary/30 hover:bg-card/80 transition-all duration-300"
              >
                <h3 className="text-2xl font-bold mb-4">{useCase.title}</h3>
                <p className="text-muted-foreground mb-6">{useCase.description}</p>
                <div className="space-y-3">
                  {useCase.benefits.map((benefit, idx) => (
                    <div key={idx} className="flex items-center gap-3">
                      <CheckCircle className="w-5 h-5 text-primary flex-shrink-0" />
                      <span className="text-sm">{benefit}</span>
                    </div>
                  ))}
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-24 bg-gradient-to-br from-primary/5 via-background to-secondary/5">
        <div className="container mx-auto px-4 text-center">
          <motion.div
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8 }}
            viewport={{ once: true }}
            className="max-w-3xl mx-auto"
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Ready to Transform Your Business with AI?
            </h2>
            <p className="text-xl text-muted-foreground mb-8">
              Let&apos;s discuss how our AI-powered solutions can drive innovation and growth for your organization.
            </p>
            <Button 
              size="lg"
              className="bg-gradient-to-r from-primary via-primary/90 to-primary/80 hover:from-primary/90 hover:to-primary text-primary-foreground font-semibold px-10 py-4 rounded-full"
            >
              <a 
                href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27m%20interested%20in%20AI-Powered%20Solutions.%20Let%27s%20schedule%20a%20consultation!"
                target="_blank"
                rel="noopener noreferrer"
                className="flex items-center gap-2"
              >
                Schedule Free Consultation
                <MessageSquare className="w-4 h-4" />
              </a>
            </Button>
          </motion.div>
        </div>
      </section>

      <FooterSection />
    </div>
  );
}
