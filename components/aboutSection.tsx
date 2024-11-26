export default function AboutSection() {
  return (
    <section
      id="about"
      className="w-full pb-12 pt-10 md:pb-24 lg:pb-32 md:pt-20 lg:pt-26 flex justify-center"
    >
      <div className="container grid items-center justify-center gap-4 px-4 text-center md:px-6">
        <div className="flex flex-col items-center justify-center space-y-4 text-center">
          <div className="space-y-2">
            <h2 className="text-3xl font-bold tracking-tighter sm:text-5xl">
              About Aura Digital Labs
            </h2>
            <p className="max-w-[900px] text-muted-foreground md:text-xl/relaxed lg:text-base/relaxed xl:text-xl/relaxed">
              Aura Digital Labs is a leading digital transformation consultancy,
              dedicated to empowering businesses of all sizes to thrive in the
              digital age. With a team of seasoned experts, we combine
              cutting-edge technology, strategic insights, and a
              customer-centric approach to deliver innovative solutions that
              drive growth and success.
            </p>
          </div>
        </div>
        <div className="mx-auto grid items-center justify-center gap-8 sm:max-w-4xl sm:grid-cols-2 md:gap-12 lg:max-w-5xl lg:grid-cols-3">
          <div className="grid gap-1">
            <h3 className="text-lg font-bold">Our Mission</h3>
            <p className="text-sm text-muted-foreground">
              To be the trusted partner in our clients&apos; digital
              transformation journey, enabling them to stay ahead of the curve
              and achieve their business goals.
            </p>
          </div>
          <div className="grid gap-1">
            <h3 className="text-lg font-bold">Our Values</h3>
            <p className="text-sm text-muted-foreground">
              Integrity, Innovation, Collaboration, and Customer Success are the
              core values that guide our every decision and action.
            </p>
          </div>
          <div className="grid gap-1">
            <h3 className="text-lg font-bold">Our History</h3>
            <p className="text-sm text-muted-foreground">
              Aura Digital Labs was founded in 2021 with a vision to
              revolutionize the way businesses leverage technology. Over the
              years, we have grown to become a trusted partner for organizations
              across various industries.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}
