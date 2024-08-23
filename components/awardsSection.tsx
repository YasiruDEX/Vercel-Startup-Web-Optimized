export default function AwardsSection() {
  return (
    <section
      id="awards"
      className="pt-20 pb-10 bg-muted flex justify-center mb-10"
    >
      <div className="container px-4 px-3 sm:px-3 md:px-6 mx-auto max-w-7xl">
        <div className="space-y-6 text-center pb-5">
          <h2 className="text-4xl font-bold mb-12">Honors & Awards</h2>
          <p className="mx-auto max-w-2xl text-muted-foreground md:text-xl lg:text-base">
            Recognizing excellence in our contributions. Our team has been
            honored with numerous awards that highlight our commitment.
          </p>
        </div>
        <div className="grid gap-8">
          {/* Award 1 */}
          <div className="relative grid grid-cols-1 md:grid-cols-[100px_1fr_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg bg-background transition-transform duration-300 ease-in-out hover:scale-105">
            <img
              src="/thropy.jpg"
              alt="Award Trophy"
              className="w-16 h-16 justify-self-center relative z-10"
            />
            <div className="border-r-4 border-black px-4 flex flex-col justify-center">
              <h3 className="text-xl font-semibold text-center">
                Championship - SLIoT Challenge
              </h3>
            </div>
            <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center">
              <li>Project Name: Project Hydrolink</li>
              <li>All island Internet of Things competition</li>
            </ul>
          </div>

          {/* Award 2 */}
          <div className="relative grid grid-cols-1 md:grid-cols-[100px_1fr_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg bg-background transition-transform duration-300 ease-in-out hover:scale-105">
            <img
              src="/thropy.jpg"
              alt="Award Trophy"
              className="w-16 h-16 justify-self-center relative z-10"
            />
            <div className="border-r-4 border-black px-4 flex flex-col justify-center">
              <h3 className="text-xl font-semibold text-center">
                Championship - Sri Lanka Arduino Challenge
              </h3>
            </div>
            <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center">
              <li>Project Name: Project Replace</li>
              <li>IEEE Challenge sphere</li>
            </ul>
          </div>

          {/* Award 3 */}
          <div className="relative grid grid-cols-1 md:grid-cols-[100px_1fr_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg bg-background transition-transform duration-300 ease-in-out hover:scale-105">
            <img
              src="/thropy.jpg"
              alt="Award Trophy"
              className="w-16 h-16 justify-self-center relative z-10"
            />
            <div className="border-r-4 border-black px-4 flex flex-col justify-center">
              <h3 className="text-xl font-semibold text-center">
                1st Runnersup - Brainstorm
              </h3>
            </div>
            <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center">
              <li>Project Name: Project Steer-Safe</li>
              <li>Healthcare Innovation Competition</li>
            </ul>
          </div>

          {/* Award 4 */}
          <div className="relative grid grid-cols-1 md:grid-cols-[100px_1fr_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg bg-background transition-transform duration-300 ease-in-out hover:scale-105">
            <img
              src="/thropy.jpg"
              alt="Award Trophy"
              className="w-16 h-16 justify-self-center relative z-10"
            />
            <div className="border-r-4 border-black px-4 flex flex-col justify-center">
              <h3 className="text-xl font-semibold text-center">
                1st Runnersup - Aurora
              </h3>
            </div>
            <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center">
              <li>Project Name: Project Face Canvas</li>
              <li>AI Ideathlon</li>
            </ul>
          </div>
        </div>
      </div>
    </section>
  );
}
