import { Avatar, AvatarImage, AvatarFallback } from "@/components/ui/avatar";

export default function TeamSection() {
  return (
    <section id="team" className="w-full py-12 md:py-24 lg:py-32">
      <div className="container grid gap-8 px-4 md:px-6 mx-auto">
        <div className="space-y-3 text-center">
          <h2 className="text-3xl font-bold tracking-tighter sm:text-4xl md:text-5xl">
            Our Team
          </h2>
          <p className="mx-auto max-w-[700px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
            Meet the talented individuals behind our success. Electronic and
            Telecommunications Engineering Undergraduate University of Moratuwa.
          </p>
        </div>
        <div className="container grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 px-4 md:px-6">
          {/* Team Member 1 */}
          <a
            href="https://www.linkedin.com/in/yasirubasnayake"
            target="_blank"
            rel="noopener noreferrer" 
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/yasiru.jpg"
                alt="Yasiru Basnayake"
                className="rounded-md"
              />
              <AvatarFallback>AP</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Yasiru Basnayake</h3>
              <p className="font-medium">Chief Operating Officer</p>
              <p className="text-muted-foreground">
                Electronic and Telecommunications Engineering Undergraduate
              </p>
            </div>
          </a>

          {/* Team Member 2 */}
          <a
            href="https://www.linkedin.com/in/prabath-wijethilaka-4950b220b/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/prabath.jpg"
                alt="Prabath Wijethilaka"
                className="rounded-md"
              />
              <AvatarFallback>DJ</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Prabath Wijethilaka</h3>
              <p className="font-medium">Chief Operating Officer</p>
              <p className="text-muted-foreground">
                Electronic and Telecommunications Engineering Undergraduate
              </p>
            </div>
          </a>

          {/* Team Member 4 */}
          <a
            href="https://www.linkedin.com/in/lasith-haputhanthri-b2919a265/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/Lasith.jpg"
                alt="Lasith Haputhantri"
                className="rounded-md"
              />
              <AvatarFallback>TJ</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Lasith Haputhantri</h3>
              <p className="font-medium">Chief Technology Officer</p>
              <p className="text-muted-foreground">
                Electronic and Telecommunications Engineering Undergraduate
              </p>
            </div>
          </a>

          {/* Team Member 3 */}
          <a
            href="https://www.linkedin.com/in/anushka-sandeepa-samaranayake-70536a227/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/Anushka.jpg"
                alt="Anushka Samaranayake"
                className="rounded-md"
              />
              <AvatarFallback>KR</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Anushka Samaranayake</h3>
              <p className="font-medium">Marketing Manager</p>
              <p className="text-muted-foreground">
                Electronic and Telecommunications Engineering Undergraduate
              </p>
            </div>
          </a>
        </div>

        <div className="container grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8 px-4 md:px-6 mt-8">

          {/* Team Member 6 */}
          <a
            href="https://www.linkedin.com/in/wimukthi-madushan-bandara/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/wimukthi.jpg"
                alt="Wimukthi Bandara"
                className="rounded-md"
              />
              <AvatarFallback>KP</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Wimukthi Bandara</h3>
              <p className="font-medium">HR Manager</p>
              <p className="text-muted-foreground">
                Computer Science Engineering Undergraduate
              </p>
            </div>
          </a>

          {/* Team Member 7 */}
          <a
            href="https://www.linkedin.com/in/sajitha-madugalle-2a2172241/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/sajitha2.jpg"
                alt="Sajitha Madugalle"
                className="rounded-md"
              />
              <AvatarFallback>NP</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Sajitha Madugalle</h3>
              <p className="font-medium">Chief Financial Manager</p>
              <p className="text-muted-foreground">
                Bio Medical Engineering Undergraduate
              </p>
            </div>
          </a>

          {/* Team Member 8 */}
          <a
            href="https://www.linkedin.com/in/dinujaya3d/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/Dinujaya.jpg"
                alt="Dinujaya Wijewickrama"
                className="rounded-md"
              />
              <AvatarFallback>SJ</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Dinujaya Wijewickrama</h3>
              <p className="font-medium">Project Manager</p>
              <p className="text-muted-foreground">
                Electronic and Telecommunications Engineering Undergraduate
              </p>
            </div>
          </a>
          
          {/* Team Member 5 */}
          <a
            href="https://www.linkedin.com/in/tashin-kavishan-09908126a/"
            target="_blank"
            rel="noopener noreferrer"
            className="flex flex-col items-center justify-center space-y-4 rounded-lg bg-background p-6 shadow-lg transition-transform duration-300 ease-in-out hover:-translate-y-2 hover:shadow-xl hover:border-2 hover:border-black"
          >
            <Avatar className="h-20 w-20">
              <AvatarImage
                src="/tashin.jpg"
                alt="Tashin Kavishan"
                className="rounded-md"
              />
              <AvatarFallback>HW</AvatarFallback>
            </Avatar>
            <div className="grid gap-1 text-center">
              <h3 className="text-xl font-bold">Tashin Kavishan</h3>
              <p className="font-medium">Project Manager</p>
              <p className="text-muted-foreground">
                Bio Medical Engineering Undergraduate
              </p>
            </div>
          </a>
        </div>
      </div>
      <div className="text-center mt-8 text-muted-foreground font-medium">
      and 10+ more professionals...
      </div>
    </section>
  );
}
