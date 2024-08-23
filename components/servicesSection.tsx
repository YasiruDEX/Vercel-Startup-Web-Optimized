import { CloudIcon, CodeIcon, DatabaseIcon, MailIcon, MapPinIcon, MountainIcon, PhoneIcon, PowerIcon, RocketIcon, SmartphoneIcon, ComputerIcon, InfoIcon, ArrowUpIcon, ArrowRightIcon} from "@/components/Icons/icons"
import {
    FaRobot,
    FaCamera,
    FaCode,
    FaWifi,
    FaBrush,
    FaMicrochip,
  } from "react-icons/fa";
  
export default function ServicesSection() {
  return (
    <section id="services" className="pt-20 pb-10 bg-muted flex justify-center">
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
          {/* Service 1 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
            <FaRobot className="h-10 w-10" />
            <h3 className="text-lg font-bold">Robotics</h3>
            <p className="text-sm text-muted-foreground">
              Advanced robotic solutions that automate tasks and enhance
              productivity.
            </p>
          </div>
          {/* Service 2 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
            <FaCamera className="h-10 w-10" />
            <h3 className="text-lg font-bold">Machine Vision</h3>
            <p className="text-sm text-muted-foreground">
              Precision vision systems for automated inspection, identification,
              and control.
            </p>
          </div>
          {/* Service 3 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
            <FaCode className="h-10 w-10" />
            <h3 className="text-lg font-bold">Software Development</h3>
            <p className="text-sm text-muted-foreground">
              Custom software solutions designed to optimize performance and
              scalability.
            </p>
          </div>
          {/* Service 4 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
            <FaWifi className="h-10 w-10" />
            <h3 className="text-lg font-bold">IoT Solutions</h3>
            <p className="text-sm text-muted-foreground">
              Connected devices and systems that streamline operations and
              enhance user experiences.
            </p>
          </div>
          {/* Service 5 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
            <FaBrush className="h-10 w-10" />
            <h3 className="text-lg font-bold">Digital Design</h3>
            <p className="text-sm text-muted-foreground">
              Innovative digital designs that captivate and engage audiences.
            </p>
          </div>
          {/* Service 6 */}
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all duration-300 ease-in-out hover:invert">
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
  );
}
