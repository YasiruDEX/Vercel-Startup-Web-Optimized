import { useState } from 'react';

export default function AwardsSection() {
  const [showAll, setShowAll] = useState(false);

  const awards = [
    {
      title: "Championship - SLIoT Challenge",
      project: "Project Hydrolink",
      description: "All island Internet of Things competition",
    },
    {
      title: "Championship - Sri Lanka Circuit Challenge",
      project: "Project Steer-Safe",
      description: "IEEE Challenge Sphere",
    },
    {
      title: "Championship - Sri Lanka Arduino Challenge",
      project: "Project Replace",
      description: "IEEE Challenge sphere",
    },
    {
      title: "1st Runnersup - Brainstorm",
      project: "Project Steer-Safe",
      description: "Healthcare Innovation Competition",
    },
    {
      title: "1st Runnersup - Sri Lanka Circuit Challenge",
      project: "Project BlindGuide",
      description: "IEEE Challenge Sphere",
    },
    {
      title: "1st Runnersup - Sri Lanka AI Challenge",
      project: "Project ElectoBot",
      description: "IEEE Challenge Sphere",
    },
    {
      title: "1st Runnersup - Aurora",
      project: "Project Face Canvas",
      description: "AI Ideathlon",
    },
  ];

  return (
    <section
      id="awards"
      className="pt-20 pb-10 bg-muted flex justify-center mb-10"
    >
      <div className="container px-4 sm:px-3 md:px-6 mx-auto max-w-7xl">
        <div className="space-y-6 text-center pb-5">
          <h2 className="text-4xl font-bold mb-12">Honors & Awards</h2>
          <p className="mx-auto max-w-2xl text-muted-foreground md:text-xl lg:text-base">
            Recognizing excellence in our contributions. Our team has been
            honored with numerous awards that highlight our commitment.
          </p>
        </div>
        <div className="relative grid gap-8">
          {/* Award Cards */}
          {(showAll ? awards : awards.slice(0, 3)).map((award, index) => (
            <div
              key={index}
              className="relative grid grid-cols-1 md:grid-cols-[100px_1fr_1fr] gap-4 p-5 bg-white shadow-lg rounded-lg bg-background transition-all duration-300 ease-in-out hover:scale-105 hover:z-10 hover:transform hover:translate-y-[-20px] hover:translate-x-[10px]"
            >
              <img
                src="/thropy.png"
                alt="Award Trophy"
                className="w-16 h-16 justify-self-center relative z-10"
              />
              <div className="border-r-4 border-black px-4 flex flex-col justify-center">
                <h3 className="text-xl font-semibold text-center">
                  {award.title}
                </h3>
              </div>
              <ul className="list-disc text-muted-foreground pl-4 flex flex-col justify-center">
                <li>Project Name: {award.project}</li>
                <li>{award.description}</li>
              </ul>
            </div>
          ))}

          {/* Show More Button */}
          <div className="text-center mt-4">
            <button
              onClick={() => setShowAll(!showAll)}
              className="px-6 py-2 bg-black text-white rounded-lg hover:bg-gray-600 transition-all"
            >
              {showAll ? "Show Less" : "Show More"}
            </button>
          </div>
        </div>
      </div>
    </section>
  );
}