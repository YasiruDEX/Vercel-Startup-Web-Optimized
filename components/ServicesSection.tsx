import { CloudIcon, CodeIcon, DatabaseIcon, SmartphoneIcon, PowerIcon, RocketIcon } from './Icons';

export default function ServicesSection() {
  return (
    <section id="services" className="py-20 bg-muted flex justify-center">
      <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
        <div className="space-y-3 mx-auto max-w-[800px]">
          <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">Our Services</h2>
          <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
            Aura Digital Labs offers a comprehensive suite of digital transformation services to help your business thrive in the digital age.
          </p>
        </div>
        <div className="mx-auto grid max-w-5xl grid-cols-1 gap-6 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4">
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <CodeIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">Web Development</h3>
            <p className="text-sm text-muted-foreground">
              Cutting-edge web solutions that drive engagement and conversion.
            </p>
          </div>
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <SmartphoneIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">Mobile App Development</h3>
            <p className="text-sm text-muted-foreground">
              Innovative mobile apps that deliver seamless user experiences.
            </p>
          </div>
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <DatabaseIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">Data Analytics</h3>
            <p className="text-sm text-muted-foreground">Actionable insights to drive strategic decisions.</p>
          </div>
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <CloudIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">Cloud Solutions</h3>
            <p className="text-sm text-muted-foreground">Scalable and secure cloud services to meet your needs.</p>
          </div>
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <RocketIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">Digital Marketing</h3>
            <p className="text-sm text-muted-foreground">
              Strategies to elevate your brand and drive online success.
            </p>
          </div>
          <div className="flex flex-col items-center justify-center gap-4 rounded-lg bg-background p-6 shadow-sm transition-all hover:bg-accent hover:text-accent-foreground">
            <PowerIcon className="h-10 w-10" />
            <h3 className="text-lg font-bold">IT Consulting</h3>
            <p className="text-sm text-muted-foreground">
              Expert advice to navigate complex IT challenges.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}
