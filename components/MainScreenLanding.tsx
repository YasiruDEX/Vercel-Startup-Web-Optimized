import Link from "next/link";
import { Avatar, AvatarImage, AvatarFallback } from "@/components/ui/avatar";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Button } from "@/components/ui/button";
import {
  FaRobot,
  FaCamera,
  FaCode,
  FaWifi,
  FaBrush,
  FaMicrochip,
} from "react-icons/fa";
import Projects from "@/components/Projects/Projects";

export function MainScreenLanding() {
  return (
    <div className="flex flex-col min-h-dvh">
      <header className="fixed top-0 left-0 right-0 px-4 lg:px-6 h-14 flex items-center backdrop-blur-md bg-white/20 shadow-sm z-50">
        <Link
          href="#"
          className="flex items-center justify-center font-bold text-lg lg:text-xl pt-4 pl-0"
        >
          <span className="sr-only">Aura Digital Labs</span>
          <img
            src="/logo.png"
            width="60"
            height="40"
            alt="Hero"
            className="mx-auto overflow-hidden rounded-t-xl object-cover pb-4"
          />
        </Link>
        <nav className="ml-auto flex gap-4 sm:gap-6">
          <Link
            href="#home"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            Home
          </Link>
          <Link
            href="#about"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            About
          </Link>
          <Link
            href="#services"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            Services
          </Link>
          <Link
            href="#projects"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            Projects
          </Link>
          <Link
            href="#team"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            Team
          </Link>
          <Link
            href="#contact"
            className="text-sm font-semibold hover:underline underline-offset-4 mt-1"
            prefetch={false}
          >
            Contact
          </Link>
          <Button
            className="h-7 px-3 rounded-lg bg-primary/30 text-primary-foreground font-medium transition-colors hover:bg-primary/70 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
            style={{ borderRadius: "30px", overflow: "hidden" }}
          >
            Get Started
          </Button>
        </nav>
      </header>

      <section id="home" className="w-full pt-12 md:pt-16 lg:pt-20 border-y">
        <div className="px-4 md:px-6 space-y-10 xl:space-y-16">
          <div className="grid max-w-[1300px] mx-auto gap-4 px-4 sm:px-6 md:px-10 md:grid-cols-2 md:gap-16 mt-10">
            <div>
              <div
                className="h-6 px-3 bg-primary/10 text-black font-extralight mb-3 inline-block"
                style={{ borderRadius: "30px", overflow: "hidden" }}
              >
                We transform your dreams
              </div>
              <h1 className="lg:leading-tighter text-3xl font-bold tracking-tighter sm:text-4xl md:text-5xl xl:text-[3.4rem] 2xl:text-[3.75rem]">
                Unleash the Power of Digital Transformation
              </h1>
              <p className="mx-auto max-w-[700px] text-muted-foreground md:text-xl pt-4">
                Aura Digital Labs is your trusted partner in navigating the
                digital landscape. We empower businesses to thrive in the
                ever-evolving digital era.
              </p>
              <div className="flex items-left w-full py-10">
                <div className="flex items-rights gap-4 w-full max-w-md">
                  <Input
                    type="text"
                    placeholder="Hi there, How can we help you?"
                    className="flex-1 h-12 px-4 border border-input focus:ring-1 focus:ring-primary focus:border-primary"
                    style={{ borderRadius: "30px", overflow: "hidden" }}
                  />
                  <Button
                    className="h-12 px-5 bg-primary text-primary-foreground font-medium transition-colors hover:bg-primary/90 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
                    style={{ borderRadius: "30px", overflow: "hidden" }}
                  >
                    Get Started
                  </Button>
                </div>
              </div>
            </div>

            <img
              src="/banner_main.jpg"
              width="1270"
              height="600"
              alt="Hero"
              style={{ borderRadius: "10px", overflow: "hidden" }}
              className="mx-auto aspect-[16/10] overflow-hidden object-cover pb-4 transition-transform transition-filter duration-1000 ease-in-out hover:scale-105 hover:brightness-105"
            />
          </div>
        </div>
      </section>
      <section
        id="about"
        className="w-full py-12 md:py-24 lg:py-32 flex justify-center"
      >
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
          <div className="flex flex-col items-center justify-center space-y-4 text-center">
            <div className="space-y-2">
              <h2 className="text-3xl font-bold tracking-tighter sm:text-5xl">
                About Aura Digital Labs
              </h2>
              <p className="max-w-[900px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
                Aura Digital Labs is a leading digital transformation
                consultancy, dedicated to empowering businesses of all sizes to
                thrive in the digital age. With a team of seasoned experts, we
                combine cutting-edge technology, strategic insights, and a
                customer-centric approach to deliver innovative solutions that
                drive growth and success.
              </p>
            </div>
          </div>
          <div className="mx-auto grid items-center justify-center gap-8 sm:max-w-4xl sm:grid-cols-2 md:gap-12 lg:max-w-5xl lg:grid-cols-3">
            <div className="grid gap-1">
              <h3 className="text-lg font-bold">Our Mission</h3>
              <p className="text-sm text-muted-foreground">
                To be the trusted partner in our clients' digital transformation
                journey, enabling them to stay ahead of the curve and achieve
                their business goals.
              </p>
            </div>
            <div className="grid gap-1">
              <h3 className="text-lg font-bold">Our Values</h3>
              <p className="text-sm text-muted-foreground">
                Integrity, Innovation, Collaboration, and Customer Success are
                the core values that guide our every decision and action.
              </p>
            </div>
            <div className="grid gap-1">
              <h3 className="text-lg font-bold">Our History</h3>
              <p className="text-sm text-muted-foreground">
                Aura Digital Labs was founded in 2021 with a vision to
                revolutionize the way businesses leverage technology. Over the
                years, we have grown to become a trusted partner for
                organizations across various industries.
              </p>
            </div>
          </div>
        </div>
      </section>

      <section
        id="services"
        className="pt-20 pb-10 bg-muted flex justify-center"
      >
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
          <div className="space-y-3 mx-auto max-w-[800px]">
            <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">
              Our Services
            </h2>
            <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Aura Digital Labs offers a comprehensive suite of advanced
              technology services to help your business stay ahead in a rapidly
              evolving digital landscape.
            </p>
          </div>
          <div className="mx-auto grid max-w-5xl grid-cols-1 gap-6 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaRobot className="h-10 w-10" />
              <h3 className="text-lg font-bold">Robotics</h3>
              <p className="text-sm text-muted-foreground">
                Advanced robotic solutions that automate tasks and enhance
                productivity.
              </p>
            </div>
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaCamera className="h-10 w-10" />
              <h3 className="text-lg font-bold">Machine Vision</h3>
              <p className="text-sm text-muted-foreground">
                Precision vision systems for automated inspection,
                identification, and control.
              </p>
            </div>
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaCode className="h-10 w-10" />
              <h3 className="text-lg font-bold">Software Development</h3>
              <p className="text-sm text-muted-foreground">
                Custom software solutions designed to optimize performance and
                scalability.
              </p>
            </div>
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaWifi className="h-10 w-10" />
              <h3 className="text-lg font-bold">IoT Solutions</h3>
              <p className="text-sm text-muted-foreground">
                Connected devices and systems that streamline operations and
                enhance user experiences.
              </p>
            </div>
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaBrush className="h-10 w-10" />
              <h3 className="text-lg font-bold">Digital Design</h3>
              <p className="text-sm text-muted-foreground">
                Innovative digital designs that captivate and engage audiences.
              </p>
            </div>
            <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
              <FaMicrochip className="h-10 w-10" />
              <h3 className="text-lg font-bold">Machine Learning & AI</h3>
              <p className="text-sm text-muted-foreground">
                Intelligent algorithms that drive data-driven decisions and
                automation.
              </p>
            </div>
          </div>
        </div>
      </section>

      <section id="projects" className="pb-20 pt-20 bg-background">
        <div className="container mx-auto">
          <h2 className="text-4xl font-bold mb-12 text-center">Our Projects</h2>
          <div className="container grid gap-6 px-4 md:grid-cols-2 lg:grid-cols-3 md:gap-8 md:px-6">
            <Projects
              description={
                "HydroLink is an IoT device that transforms any domestic water tank into a smart one."
              }
              title={"Project Hydrolink"}
              more_details={"Hydrolink is designed to convert any existing water tank into a smart, efficient system, providing real-time monitoring and control without causing any damage to the existing structure like never before. For more details visit www.hydrolink.lk"}
              technologies={"Google Firebase, ESP Microcontroller, Altium Designer, Arduino, SolidWorks, Flutter"}
              // source={"/hydrolink.mp4"}
              image1={"/hydrolink2.png"}
              image2={"/hydrolink4.png"}
              image3={"/hydrolink3.png"}
          
            />

            <Projects
              description={
                "A wearable device that utilizes machine learning and Electrooculography (EOG) signals to track a driver’s state of awareness in real-time"}
              title={"Project Steer-Safe"}
              more_details={"Steer Safe is a device which utilizes machine learning and electrooculography (EOG) signals to track a driver's state of awareness in real-time, providing early alerts and potentially saving lives."}
              technologies={"Analog Filter, Radio Frequncy Communication, High Speed PCB Design, Raspberrypi, wearable biomedical device design, Machine learning(Classification) "}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />

            <Projects
              description={
                "Vision-based Automated Restaurant Robot is a cutting-edge project"
              }
              title={"Project Luna"}
              more_details={"LUNA is Vision-based Automated Restaurant Robot is a cutting-edge project. This robot uses ROS and Kinect2 to navigate and interact autonomously within restaurant environments. It features a custom Kalman-based food stabilization tray designed to prevent beverage spillage, enhancing operational efficiency. The system integrates advanced sensor technologies and robotic functionalities, complemented by a custom restaurant environment planner front end for seamless interaction."}
              technologies={"ROS (Noetic, Iron), Kinect 2 depth camera, Rtabmap, Turtlebot, Kalman, Ubuntu 22.04 LTS, Atmel Microchip Studio, CMake, libfreenect2, Localization, PID, PCB Design, SLAM, Python, SolidWorks"}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />

            <Projects
              description={
                "Replace is smart power outlet that can monitor and control the power usage of appliences intergrated with ML ."
              }
              title={"Project Replace"}
              more_details={"Replace is an innovative system designed to interface with existing domestic electrical systems in Sri Lanka. It monitors the power usage of individual outlets, allowing for real-time tracking and management of energy consumption. By integrating the outlets with a home automation system, users can remotely control and optimize their energy usage, enhancing both convenience and efficiency. Additionally, Replace utilizes machine learning with autoencoders to detect anomalies in current and voltage waveforms, enabling early identification of potential issues and protecting devices from potential damage"}
              technologies={"Django, React Native, Firebase, Analog Circuit Design, Altium Designer, LTspice, NI Multisim ,Autoencoders (Machine Learning)"}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />


            <Projects
              description={
                "Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking"
              }
              title={"Project Earendel Pro-Track"}
              more_details={"Earendel Pro-Track is an IoT-based Alt Azimuth telescope mount designed for automated celestial tracking. This device enhances the stargazing experience by allowing users to track distant celestial objects with precision. It features real-time updates, user-friendly controls, and educational value, making it an affordable solution for astronomy enthusiasts. Earendel Pro-Track includes a separate mobile app and web interface for easy control and management of the telescope, ensuring an accessible and interactive experience."}
              technologies={"Equatorial Mount Design, MEMS, PID, Mathematical Models, Celestial Databases, Mobile/Web App Development, PCB Design"}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />



            <Projects
              description={
                "The Stable Diffusion Based Criminal Face Generation Platform is an AI-driven tool that automates the creation of realistic criminal facial images from forensic data, improving law enforcement's accuracy and efficiency in suspect identification."
              }
              title={"Project face canvas"}
              more_details={"The Stable Diffusion Based Criminal Face Generation Platform is an advanced AI-driven solution that leverages stable diffusion models to generate accurate and realistic criminal facial images from forensic data. This platform automates the facial synthesis process, reducing reliance on manual sketching and minimizing subjective bias. By enhancing law enforcement's ability to accurately identify suspects, the platform contributes to more efficient and reliable forensic investigations."}
              technologies={"LLMs, PyTorch, Stable Diffusion models, Hugging Face Transformers, ONNX"}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />




            <Projects
              description={
                "This device measures key water quality parameters and provides feedback on water suitability based on standard benchmarks."
              }
              title={"Industrial Portable Water Quality measuring Device"}
              more_details={"This device measures water quality parameters such as pH, conductivity, temperature, and turbidity. It analyzes these values against standard benchmarks and provides user feedback on whether the water is suitable for use."}
              technologies={"Sensor Technology, PCB Design with test points, Mouldable Enclousure Design, Google Firebase, Flutter"}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            />



            {/* <Projects
              description={
                ""
              }
              title={""}
              more_details={""}
              technologies={""}
              // source={"/hydrolink.mp4"}
              image1={"/product.png"}
              image2={"/product.png"}
              image3={"/product.png"}

            /> */}


            {/* 
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              
              <div className="group relative w-full h-full">
                <video
                  src="/hydrolink.mp4"
                  width={400}
                  height={400}
                  className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                  style={{ aspectRatio: "400/400", objectFit: "cover" }}
                  muted
                  loop
                  preload="metadata"
                  onLoadedMetadata={(e) => e.target.play()}
                  onMouseEnter={(e) => e.target.play()}
                  onMouseLeave={(e) => {
                    e.target.pause();
                    e.target.currentTime = 0;
                  }}
                />
              </div>

              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">
                  Hydrolink: Smart Water Tank
                </h3>
                <p className="text-sm text-muted-foreground">
                  HydroLink is an IoT device that transforms any domestic water
                  tank into a smart one.
                </p>
              </div>
            </div>

            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              <img
                src="/product.png"
                alt="TechBot"
                width={400}
                height={400}
                className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                style={{ aspectRatio: "400/400", objectFit: "cover" }}
              />
              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">TechBot</h3>
                <p className="text-sm text-muted-foreground">
                  STM32-based competition robot with machine vision and custom
                  PCB.
                </p>
              </div>
            </div>

            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              <img
                src="/product.png"
                alt="Cosmo Robot"
                width={400}
                height={400}
                className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                style={{ aspectRatio: "400/400", objectFit: "cover" }}
              />
              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">Cosmo Robot</h3>
                <p className="text-sm text-muted-foreground">
                  Multi-functional robot with ATMEGA2560, featuring navigation
                  and obstacle avoidance.
                </p>
              </div>
            </div>

            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              <img
                src="/product.png"
                alt="Criminal Face Generation"
                width={400}
                height={400}
                className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                style={{ aspectRatio: "400/400", objectFit: "cover" }}
              />
              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">
                  Criminal Face Generation
                </h3>
                <p className="text-sm text-muted-foreground">
                  AI-driven platform for forensic facial synthesis using stable
                  diffusion models.
                </p>
              </div>
            </div>

            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              <img
                src="/product.png"
                alt="Vision-Language Navigation"
                width={400}
                height={400}
                className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                style={{ aspectRatio: "400/400", objectFit: "cover" }}
              />
              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">
                  Vision-Language Navigation
                </h3>
                <p className="text-sm text-muted-foreground">
                  SLAM system using vision-language inputs for robot navigation.
                </p>
              </div>
            </div>

            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View</span>
              </Link>
              <img
                src="/product.png"
                alt="Ratatouille Micromouse"
                width={400}
                height={400}
                className="h-full w-full transition-transform duration-300 ease-in-out group-hover:scale-105 group-hover:opacity-90 object-cover"
                style={{ aspectRatio: "400/400", objectFit: "cover" }}
              />
              <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100">
                <h3 className="text-lg font-semibold">
                  Ratatouille: Micromouse
                </h3>
                <p className="text-sm text-muted-foreground">
                  Fast micromouse for autonomous maze navigation using advanced
                  sensors.
                </p>
              </div>
            </div>
            */}
          </div>
        </div>
      </section>

      <section id="awards" className="w-full py-12 md:py-24 lg:py-32">
        <div className="container grid gap-8 px-4 md:px-6 mx-auto">
          <div className="space-y-3">
            <h2 className="text-4xl font-bold mb-12 text-center">Awards</h2>
            <p className="mx-auto max-w-[700px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Recognizing excellence in our projects and contributions.
            </p>
          </div>

          <div className="space-y-8">
            {/* Award 1 */}
            <div className="grid grid-cols-[100px_600px_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg">
              <img
                src="/thropy.jpg"
                alt="Award Trophy"
                className="w-16 h-16 justify-self-center"
              />
              <div className="self-center border-r-4 border-black px-4">
                <h3 className="text-xl font-semibold text-center">
                  Championship - SLIoT Challenge
                </h3>
              </div>
              <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center border-black">
                <li>Project Name: Project Hydrolink</li>
                <li>All island Internet of Things competition</li>
              </ul>
            </div>

            {/* Award 2 */}
            <div className="grid grid-cols-[100px_600px_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg">
              <img
                src="/thropy.jpg"
                alt="Award Trophy"
                className="w-16 h-16 justify-self-center"
              />
              <div className="self-center border-r-4 border-black px-4">
                <h3 className="text-xl font-semibold text-center">
                  Championship - Sri Lanka Arduino Challenge
                </h3>
              </div>
              <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center border-black">
                <li>Project Name: Project Replace</li>
                <li>IEEE Challenge sphere</li>
              </ul>
            </div>

            {/* Award 3 */}
            <div className="grid grid-cols-[100px_600px_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg">
              <img
                src="/thropy.jpg"
                alt="Award Trophy"
                className="w-16 h-16 justify-self-center"
              />
              <div className="self-center border-r-4 border-black px-4">
                <h3 className="text-xl font-semibold text-center">
                  1st Runnersup - Brainstorm
                </h3>
              </div>
              <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center border-black">
                <li>Project Name: Project Steer-Safe</li>
                <li>Healthcare Innovation Competition</li>
              </ul>
            </div>

            {/* Award 4 */}
            <div className="grid grid-cols-[100px_600px_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg">
              <img
                src="/thropy.jpg"
                alt="Award Trophy"
                className="w-16 h-16 justify-self-center"
              />
              <div className="self-center border-r-4 border-black px-4">
                <h3 className="text-xl font-semibold text-center">
                  1st Runnersup - Aurora
                </h3>
              </div>
              <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center border-black">
                <li>Project Name: Project Face Canvas</li>
                <li>AI Ideathlon</li>
              </ul>
            </div>
          </div>
        </div>
      </section>

      <section id="team" className="w-full py-12 md:py-24 lg:py-32">
        <div className="container grid gap-8 px-4 md:px-6 mx-auto">
          <div className="space-y-3">
            <h2 className="text-3xl font-bold tracking-tighter sm:text-4xl md:text-5xl">
              Our Team
            </h2>
            <p className="mx-auto max-w-[700px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Meet the talented individuals behind our success. Electronic and
              Telecommunications Engineering Undergraduate University of
              Moratuwa.
            </p>
          </div>
          <div className="grid gap-6 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/yasiru.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Yasiru Basnayake</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    Robitics / AI
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/prabath.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Prabath Wijethilaka</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    FPGA / IoT
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/Anushka.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">
                    Anushka Samaranayake
                  </h3>
                  {/* <p className="text-sm text-muted-foreground">Designer</p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/Lasith.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Lasith Haputhantri</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    Marketing Specialist
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/tashin.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Tashin Kavishan</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    Software Engineer
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/wimukthi.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Wimukthi Bandara</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    Product Designer
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/sajitha.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">Sajitha Madugalle</h3>
                  {/* <p className="text-sm text-muted-foreground">
                    Data Scientist
                  </p> */}
                </div>
              </div>
            </div>
            <div className="group relative overflow-hidden rounded-lg">
              <Link href="#" className="absolute inset-0 z-10" prefetch={false}>
                <span className="sr-only">View profile</span>
              </Link>
              <div className="flex flex-col items-center justify-center gap-2 p-4 transition-all duration-300 group-hover:bg-muted">
                <div className="relative h-24 w-24 overflow-hidden rounded-full">
                  <img
                    src="/Dinujaya.jpg"
                    alt="Avatar"
                    width={96}
                    height={96}
                    className="h-full w-full object-cover rounded-lg transition-all duration-300 group-hover:scale-110 group-hover:opacity-80"
                    style={{ aspectRatio: "96/96", objectFit: "cover" }}
                  />
                </div>
                <div className="text-center">
                  <h3 className="text-lg font-semibold">
                    Dinujaya Wijewickrama
                  </h3>
                  {/* <p className="text-sm text-muted-foreground">
                    DevOps Engineer
                  </p> */}
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      <section id="contact" className="w-full py-12 md:py-24 lg:py-32 bg-muted">
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6 mx-auto">
          <div className="space-y-3">
            <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">
              Get in Touch
            </h2>
            <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Have a project in mind or need our expertise? Fill out the form
              below and we'll get back to you as soon as possible.
            </p>
          </div>
          <div className="mx-auto w-full max-w-sm space-y-4">
            <form className="grid gap-4">
              <Input
                type="text"
                placeholder="Name"
                className="max-w-lg flex-1"
              />
              <Input
                type="email"
                placeholder="Email"
                className="max-w-lg flex-1"
              />
              <Textarea placeholder="Message" className="max-w-lg flex-1" />
              <Button type="submit">Submit</Button>
            </form>
            <div className="grid gap-2">
              <div className="flex items-center gap-2">
                <PhoneIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  +1 (555) 123-4567
                </p>
              </div>
              <div className="flex items-center gap-2">
                <MailIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  info@auradigitallabs.com
                </p>
              </div>
              <div className="flex items-center gap-2">
                <MapPinIcon className="h-5 w-5 text-muted-foreground" />
                <p className="text-sm text-muted-foreground">
                  123 Main Street, Anytown USA
                </p>
              </div>
            </div>
          </div>
        </div>
      </section>

      <div className="fixed bottom-4 right-4">
        <Button
          variant="ghost"
          size="icon"
          className="bg-primary/30 text-primary-foreground hover:bg-primary/30 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring backdrop-blur-lg"
        // onClick={() => window.scrollTo({ top: 0, behavior: "smooth" })}
        >
          <ArrowUpIcon className="w-6 h-6" />
          <span className="sr-only">Scroll to top</span>
        </Button>
      </div>

      <footer className="bg-gray-100 py-8">
        <div className="container mx-auto px-4">
          <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-8">
            <div>
              <h2 className="text-lg font-bold mb-4">Company</h2>
              <ul className="space-y-2">
                <li>About Us</li>
                <li>Our Team</li>
                <li>Careers</li>
                <li>Press</li>
                <li>Blog</li>
              </ul>
            </div>
            <div>
              <h2 className="text-lg font-bold mb-4">Services</h2>
              <ul className="space-y-2">
                <li>Product Development</li>
                <li>Consulting</li>
                <li>Support</li>
                <li>Partnerships</li>
                <li>Client Resources</li>
              </ul>
            </div>
            <div>
              <h2 className="text-lg font-bold mb-4">Resources</h2>
              <ul className="space-y-2">
                <li>Documentation</li>
                <li>Tutorials</li>
                <li>FAQs</li>
                <li>Community Forum</li>
                <li>Case Studies</li>
              </ul>
            </div>
            <div>
              <h2 className="text-lg font-bold mb-4">Contact</h2>
              <ul className="space-y-2">
                <li>Contact Us</li>
                <li>Support Center</li>
                <li>Request a Demo</li>
                <li>Feedback</li>
                <li>Social Media</li>
              </ul>
            </div>
          </div>
          <div className="mt-8 text-center text-sm text-gray-600">
            <p>
              Aura Digital Labs is dedicated to delivering innovative solutions
              and exceptional service. Stay connected with us to explore how we
              can help you achieve your goals.
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
}

function CloudIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M17.5 19H9a7 7 0 1 1 6.71-9h1.79a4.5 4.5 0 1 1 0 9Z" />
    </svg>
  );
}

function CodeIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <polyline points="16 18 22 12 16 6" />
      <polyline points="8 6 2 12 8 18" />
    </svg>
  );
}

function DatabaseIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <ellipse cx="12" cy="5" rx="9" ry="3" />
      <path d="M3 5V19A9 3 0 0 0 21 19V5" />
      <path d="M3 12A9 3 0 0 0 21 12" />
    </svg>
  );
}

function MailIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect width="20" height="16" x="2" y="4" rx="2" />
      <path d="m22 7-8.97 5.7a1.94 1.94 0 0 1-2.06 0L2 7" />
    </svg>
  );
}

function MapPinIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M20 10c0 6-8 12-8 12s-8-6-8-12a8 8 0 0 1 16 0Z" />
      <circle cx="12" cy="10" r="3" />
    </svg>
  );
}

function MountainIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="m8 3 4 8 5-5 5 15H2L8 3z" />
    </svg>
  );
}

function PhoneIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M22 16.92v3a2 2 0 0 1-2.18 2 19.79 19.79 0 0 1-8.63-3.07 19.5 19.5 0 0 1-6-6 19.79 19.79 0 0 1-3.07-8.67A2 2 0 0 1 4.11 2h3a2 2 0 0 1 2 1.72 12.84 12.84 0 0 0 .7 2.81 2 2 0 0 1-.45 2.11L8.09 9.91a16 16 0 0 0 6 6l1.27-1.27a2 2 0 0 1 2.11-.45 12.84 12.84 0 0 0 2.81.7A2 2 0 0 1 22 16.92z" />
    </svg>
  );
}

function PowerIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M12 2v10" />
      <path d="M18.4 6.6a9 9 0 1 1-12.77.04" />
    </svg>
  );
}

function RocketIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M4.5 16.5c-1.5 1.26-2 5-2 5s3.74-.5 5-2c.71-.84.7-2.13-.09-2.91a2.18 2.18 0 0 0-2.91-.09z" />
      <path d="m12 15-3-3a22 22 0 0 1 2-3.95A12.88 12.88 0 0 1 22 2c0 2.72-.78 7.5-6 11a22.35 22.35 0 0 1-4 2z" />
      <path d="M9 12H4s.55-3.03 2-4c1.62-1.08 5 0 5 0" />
      <path d="M12 15v5s3.03-.55 4-2c1.08-1.62 0-5 0-5" />
    </svg>
  );
}

function SmartphoneIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect width="14" height="20" x="5" y="2" rx="2" ry="2" />
      <path d="M12 18h.01" />
    </svg>
  );
}

function ComputerIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect width="14" height="8" x="5" y="2" rx="2" />
      <rect width="20" height="8" x="2" y="14" rx="2" />
      <path d="M6 18h2" />
      <path d="M12 18h6" />
    </svg>
  );
}

function InfoIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <path d="M12 16v-4" />
      <path d="M12 8h.01" />
    </svg>
  );
}

function ArrowUpIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="m5 12 7-7 7 7" />
      <path d="M12 19V5" />
    </svg>
  );
}

function ArrowRightIcon(props) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M5 12h14" />
      <path d="M12 5l7 7-7 7" />
    </svg>
  );
}
