import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";

export default function LandingIntro() {
  return (
    <section id="home" className="w-full pt-12 md:pt-16 lg:pt-20 border-y">
      <div className="px-4 md:px-6 space-y-10 xl:space-y-16">
        <div className="grid max-w-[1300px] mx-auto gap-4 px-4 sm:px-6 md:px-10 md:grid-cols-2 md:gap-16 mt-10">
          <div>
            <div
              className="h-6 px-3 bg-primary/10 text-black font-light mb-3 inline-block"
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
                <a
                  href="https://api.whatsapp.com/send/?phone=94714745349"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  <Button
                    className="h-12 px-5 bg-primary text-primary-foreground font-medium transition-colors hover:bg-primary/90 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
                    style={{ borderRadius: "30px", overflow: "hidden" }}
                  >
                    Get Started
                  </Button>
                </a>
              </div>
            </div>
          </div>
          <div className="relative w-full h-64 flex items-center justify-center">
  <div className="relative w-full h-full bg-black flex items-center justify-center">
    <img
      src="/banner_main.jpg"
      alt="Hero"
      className="absolute lg:top-10  md:top-20 inset-2 w-full h-full lg:h-100 md:h-100 sm:h-auto rounded-md object-cover transition-opacity duration-500 ease-in-out hover:opacity-0"
    />
    <img
      src="/banner_main2.png"
      alt="Hero Hover"
      className="absolute lg:top-10 md:top-20 inset-2 w-full h-full lg:h-100 md:h-100 sm:h-auto rounded-md object-cover transition-opacity opacity-0 duration-500 ease-in-out hover:opacity-100"
    />
  </div>
</div>

        </div>
      </div>
    </section>
  );
}
