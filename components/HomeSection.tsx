export default function HomeSection() {
    return (
      <section id="home" className="w-full pt-12 md:pt-24 lg:pt-32 border-y">
        <div className="px-4 md:px-6 space-y-10 xl:space-y-16">
          <div className="grid max-w-[1300px] mx-auto gap-4 px-4 sm:px-6 md:px-10 md:grid-cols-2 md:gap-16 mt-10">
            <div>
              <h1 className="lg:leading-tighter text-3xl font-bold tracking-tighter sm:text-4xl md:text-5xl xl:text-[3.4rem] 2xl:text-[3.75rem]">
                Unleash the Power of Digital Transformation
              </h1>
              <p className="mx-auto max-w-[700px] text-muted-foreground md:text-xl pt-4">
                Aura Digital Labs is your trusted partner in navigating the digital landscape. We empower businesses to thrive in the ever-evolving digital era.
              </p>
            </div>
            <img
              src="/banner_main.jpg"
              width="1270"
              height="600"
              alt="Hero"
              className="mx-auto aspect-[16/9] overflow-hidden rounded-t-xl object-cover pb-4"
            />
          </div>
        </div>
      </section>
    );
  }
  