export default function ProjectsSection() {
    return (
      <section id="projects" className="w-full py-20 bg-muted flex justify-center">
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
          <div className="space-y-3 mx-auto max-w-[800px]">
            <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">Our Projects</h2>
            <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Explore some of our most recent and impactful projects that showcase our capabilities and expertise.
            </p>
          </div>
          {/* Add project entries here */}
        </div>
      </section>
    );
  }
  