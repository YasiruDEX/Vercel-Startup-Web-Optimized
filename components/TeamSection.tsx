export default function TeamSection() {
    return (
      <section id="team" className="w-full py-20 bg-muted flex justify-center">
        <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
          <div className="space-y-3 mx-auto max-w-[800px]">
            <h2 className="text-3xl font-bold tracking-tighter md:text-4xl/tight">Meet Our Team</h2>
            <p className="mx-auto max-w-[600px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Our team of dedicated professionals brings a wealth of knowledge and experience to every project.
            </p>
          </div>
          {/* Add team member entries here */}
        </div>
      </section>
    );
  }
  