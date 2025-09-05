"use client";

import Projects from "@/components/Projects/Projects";
import { Button } from "@/components/ui/button";
import { motion, useInView } from "framer-motion";
import { useRef } from "react";

export default function ProjectsSection() {
  const ref = useRef(null);
  const isInView = useInView(ref, { once: true, margin: "-100px" });

  const projectsData = [
    {
      description: "HydroLink is an IoT device that transforms any domestic water tank into a smart one.",
      title: "Project Hydrolink",
      more_details: "Hydrolink is designed to convert any existing water tank into a smart, efficient system, providing real-time monitoring and control without causing any damage to the existing structure like never before. For more details visit www.hydrolink.lk",
      technologies: "Google Firebase, ESP Microcontroller, Altium Designer, Arduino, SolidWorks, Flutter",
      image1: "/hydrolink2.png",
      image2: "/hydrolink3.png",
      image3: "/hydrolink4.png"
    },
    {
      description: "A wearable device that utilizes machine learning and Electrooculography (EOG) signals to track a driver's state of awareness in real-time",
      title: "Project Steer-Safe",
      more_details: "Steer Safe is a device which utilizes machine learning and electrooculography (EOG) signals to track a driver's state of awareness in real-time, providing early alerts and potentially saving lives.",
      technologies: "Analog Filter, Radio Frequncy Communication, High Speed PCB Design, Raspberrypi, wearable biomedical device design, Machine learning(Classification) ",
      image1: "/Steersafe1.png",
      image2: "/Steersafe2.png",
      image3: "/Steersafe3.png"
    },
    {
      description: "Vision-based Automated Restaurant Robot is a cutting-edge project",
      title: "Project Luna",
      more_details: "LUNA is Vision-based Automated Restaurant Robot is a cutting-edge project. This robot uses ROS and Kinect2 to navigate and interact autonomously within restaurant environments. It features a custom Kalman-based food stabilization tray designed to prevent beverage spillage, enhancing operational efficiency. The system integrates advanced sensor technologies and robotic functionalities, complemented by a custom restaurant environment planner front end for seamless interaction.",
      technologies: "ROS (Noetic, Iron), Kinect 2 depth camera, Rtabmap, Turtlebot, Kalman, Ubuntu 22.04 LTS, Atmel Microchip Studio, CMake, libfreenect2, Localization, PID, PCB Design, SLAM, Python, SolidWorks",
      image1: "/19.png",
      image2: "/14.png",
      image3: "/15.png"
    },
    {
      description: "Replace is smart power outlet that can monitor and control the power usage of appliences intergrated with ML .",
      title: "Project Replace",
      more_details: "Replace is an innovative system designed to interface with existing domestic electrical systems in Sri Lanka. It monitors the power usage of individual outlets, allowing for real-time tracking and management of energy consumption. By integrating the outlets with a home automation system, users can remotely control and optimize their energy usage, enhancing both convenience and efficiency. Additionally, Replace utilizes machine learning with autoencoders to detect anomalies in current and voltage waveforms, enabling early identification of potential issues and protecting devices from potential damage",
      technologies: "Django, React Native, Firebase, Analog Circuit Design, Altium Designer, LTspice, NI Multisim ,Autoencoders (Machine Learning)",
      image1: "/7.png",
      image2: "/8.png",
      image3: "/9.png"
    },
    {
      description: "Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking",
      title: "Project Earendel Pro-Track",
      more_details: "Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking. This device enhances the stargazing experience by allowing users to track distant celestial objects with precision. It features real-time updates, user-friendly controls, and educational value, making it an affordable solution for astronomy enthusiasts. Earendel Pro-Track includes a separate mobile app and web interface for easy control and management of the telescope, ensuring an accessible and interactive experience.",
      technologies: "Equatorial Mount Design, MEMS, PID, Mathematical Models, Celestial Databases, Mobile/Web App Development, PCB Design",
      image1: "/10.png",
      image2: "/11.png",
      image3: "/12.png"
    },
    {
      description: "The Stable Diffusion Based Criminal Face Generation Platform is an AI-driven tool that automates the creation of realistic criminal facial images from forensic data, improving law enforcement's accuracy and efficiency in suspect identification.",
      title: "Project Face Canvas",
      more_details: "The Stable Diffusion Based Criminal Face Generation Platform is an advanced AI-driven solution that leverages stable diffusion models to generate accurate and realistic criminal facial images from forensic data. This platform automates the facial synthesis process, reducing reliance on manual sketching and minimizing subjective bias. By enhancing law enforcement's ability to accurately identify suspects, the platform contributes to more efficient and reliable forensic investigations.",
      technologies: "LLMs, PyTorch, Stable Diffusion models, Hugging Face Transformers, ONNX",
      image1: "/face.png",
      image2: "/27.png",
      image3: "/28.png"
    },
    {
      description: "This device measures key water quality parameters and provides feedback on water suitability based on standard benchmarks.",
      title: "Industrial Portable Water Quality Measuring Device",
      more_details: "This device measures water quality parameters such as pH, conductivity, temperature, and turbidity. It analyzes these values against standard benchmarks and provides user feedback on whether the water is suitable for use.",
      technologies: "Sensor Technology, PCB Design with test points, Mouldable Enclosure Design, Google Firebase, Flutter",
      image1: "/23.png",
      image2: "/w.png",
      image3: "/24.png"
    }
  ];

  return (
    <section ref={ref} id="projects" className="py-24 bg-gradient-to-br from-background via-muted/5 to-background relative overflow-hidden">
      {/* Enhanced background effects */}
      <div className="absolute inset-0">
        <motion.div 
          className="absolute top-1/3 left-1/4 w-96 h-96 bg-primary/3 rounded-full blur-3xl"
          animate={{ 
            scale: [1, 1.3, 1],
            opacity: [0.2, 0.4, 0.2],
            rotate: [0, 180, 360]
          }}
          transition={{ 
            duration: 20,
            repeat: Infinity,
            ease: "easeInOut"
          }}
        />
        <motion.div 
          className="absolute bottom-1/3 right-1/4 w-80 h-80 bg-secondary/4 rounded-full blur-3xl"
          animate={{ 
            scale: [1.2, 1, 1.2],
            opacity: [0.15, 0.3, 0.15],
            rotate: [360, 180, 0]
          }}
          transition={{ 
            duration: 25,
            repeat: Infinity,
            ease: "easeInOut",
            delay: 5
          }}
        />
      </div>

      <div className="container mx-auto max-w-7xl px-4 relative z-10">
        {/* Header with enhanced animations */}
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
            Our Work
          </motion.h2>
          <motion.p 
            className="mx-auto max-w-3xl text-muted-foreground text-lg md:text-xl leading-relaxed"
            initial={{ opacity: 0, y: 20 }}
            animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 20 }}
            transition={{ duration: 0.8, delay: 0.4 }}
          >
            Explore our diverse portfolio of innovative projects that showcase our expertise in cutting-edge technology solutions and real-world impact.
          </motion.p>
        </motion.div>

        {/* Projects Grid with enhanced animations */}
        <motion.div 
          className="grid gap-8 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-3"
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
          {projectsData.map((project, index) => (
            <motion.div
              key={index}
              variants={{
                hidden: { opacity: 0, y: 60, scale: 0.8 },
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
              <Projects {...project} />
            </motion.div>
          ))}
        </motion.div>

        {/* Enhanced CTA Button */}
        <motion.div 
          className="text-center mt-20"
          initial={{ opacity: 0, y: 30 }}
          animate={isInView ? { opacity: 1, y: 0 } : { opacity: 0, y: 30 }}
          transition={{ duration: 0.8, delay: 1.5 }}
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
              Discuss Your Project
            </Button>
          </motion.a>
        </motion.div>
      </div>
    </section>
  );
}
