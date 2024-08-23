import Projects from "@/components/Projects/Projects";

export default function ProjectsSection() {
  return (
    <section id="projects" className="pb-20 pt-20 bg-background">
      <div className="container mx-auto max-w-5xl">
        <h2 className="text-4xl font-bold mb-12 text-center">Our Projects</h2>
        <div className="container grid gap-6 px-4 sm:grid-cols-2 md:grid-cols-2 lg:grid-cols-3 md:gap-8 md:px-6">
          <Projects
            description={
              "HydroLink is an IoT device that transforms any domestic water tank into a smart one."
            }
            title={"Project Hydrolink"}
            more_details={
              "Hydrolink is designed to convert any existing water tank into a smart, efficient system, providing real-time monitoring and control without causing any damage to the existing structure like never before. For more details visit www.hydrolink.lk"
            }
            technologies={
              "Google Firebase, ESP Microcontroller, Altium Designer, Arduino, SolidWorks, Flutter"
            }
            image1={"/hydrolink2.png"}
            image2={"/hydrolink3.png"}
            image3={"/hydrolink4.png"}
          />

          <Projects
            description={
              "A wearable device that utilizes machine learning and Electrooculography (EOG) signals to track a driver’s state of awareness in real-time"
            }
            title={"Project Steer-Safe"}
            more_details={
              "Steer Safe is a device which utilizes machine learning and electrooculography (EOG) signals to track a driver's state of awareness in real-time, providing early alerts and potentially saving lives."
            }
            technologies={
              "Analog Filter, Radio Frequncy Communication, High Speed PCB Design, Raspberrypi, wearable biomedical device design, Machine learning(Classification) "
            }
            image1={"/Steersafe1.png"}
            image2={"/Steersafe2.png"}
            image3={"/Steersafe3.png"}
          />

          <Projects
            description={
              "Vision-based Automated Restaurant Robot is a cutting-edge project"
            }
            title={"Project Luna"}
            more_details={
              "LUNA is Vision-based Automated Restaurant Robot is a cutting-edge project. This robot uses ROS and Kinect2 to navigate and interact autonomously within restaurant environments. It features a custom Kalman-based food stabilization tray designed to prevent beverage spillage, enhancing operational efficiency. The system integrates advanced sensor technologies and robotic functionalities, complemented by a custom restaurant environment planner front end for seamless interaction."
            }
            technologies={
              "ROS (Noetic, Iron), Kinect 2 depth camera, Rtabmap, Turtlebot, Kalman, Ubuntu 22.04 LTS, Atmel Microchip Studio, CMake, libfreenect2, Localization, PID, PCB Design, SLAM, Python, SolidWorks"
            }
            image1={"/placeholder.svg"}
            image2={"/placeholder.svg"}
            image3={"/placeholder.svg"}
          />

          <Projects
            description={
              "Replace is smart power outlet that can monitor and control the power usage of appliences intergrated with ML ."
            }
            title={"Project Replace"}
            more_details={
              "Replace is an innovative system designed to interface with existing domestic electrical systems in Sri Lanka. It monitors the power usage of individual outlets, allowing for real-time tracking and management of energy consumption. By integrating the outlets with a home automation system, users can remotely control and optimize their energy usage, enhancing both convenience and efficiency. Additionally, Replace utilizes machine learning with autoencoders to detect anomalies in current and voltage waveforms, enabling early identification of potential issues and protecting devices from potential damage"
            }
            technologies={
              "Django, React Native, Firebase, Analog Circuit Design, Altium Designer, LTspice, NI Multisim ,Autoencoders (Machine Learning)"
            }
            image1={"/placeholder.svg"}
            image2={"/placeholder.svg"}
            image3={"/placeholder.svg"}
          />

          <Projects
            description={
              "Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking"
            }
            title={"Project Earendel Pro-Track"}
            more_details={
              "Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking. This device enhances the stargazing experience by allowing users to track distant celestial objects with precision. It features real-time updates, user-friendly controls, and educational value, making it an affordable solution for astronomy enthusiasts. Earendel Pro-Track includes a separate mobile app and web interface for easy control and management of the telescope, ensuring an accessible and interactive experience."
            }
            technologies={
              "Equatorial Mount Design, MEMS, PID, Mathematical Models, Celestial Databases, Mobile/Web App Development, PCB Design"
            }
            image1={"/placeholder.svg"}
            image2={"/placeholder.svg"}
            image3={"/placeholder.svg"}
          />

          <Projects
            description={
              "The Stable Diffusion Based Criminal Face Generation Platform is an AI-driven tool that automates the creation of realistic criminal facial images from forensic data, improving law enforcement's accuracy and efficiency in suspect identification."
            }
            title={"Project Face Canvas"}
            more_details={
              "The Stable Diffusion Based Criminal Face Generation Platform is an advanced AI-driven solution that leverages stable diffusion models to generate accurate and realistic criminal facial images from forensic data. This platform automates the facial synthesis process, reducing reliance on manual sketching and minimizing subjective bias. By enhancing law enforcement's ability to accurately identify suspects, the platform contributes to more efficient and reliable forensic investigations."
            }
            technologies={
              "LLMs, PyTorch, Stable Diffusion models, Hugging Face Transformers, ONNX"
            }
            image1={"/placeholder.svg"}
            image2={"/placeholder.svg"}
            image3={"/placeholder.svg"}
          />

          <Projects
            description={
              "This device measures key water quality parameters and provides feedback on water suitability based on standard benchmarks."
            }
            title={"Industrial Portable Water Quality Measuring Device"}
            more_details={
              "This device measures water quality parameters such as pH, conductivity, temperature, and turbidity. It analyzes these values against standard benchmarks and provides user feedback on whether the water is suitable for use."
            }
            technologies={
              "Sensor Technology, PCB Design with test points, Mouldable Enclosure Design, Google Firebase, Flutter"
            }
            image1={"/placeholder.svg"}
            image2={"/placeholder.svg"}
            image3={"/placeholder.svg"}
          />
        </div>
      </div>
    </section>
  );
}
