import Link from "next/link";
import { Button } from "@/components/ui/button";

export default function HeaderSection() {
  return (
    <header className="fixed top-0 left-0 right-0 px-4 lg:px-6 h-16 flex items-center justify-between backdrop-blur-md bg-white/20 shadow-sm z-50">
      <Link href="#" className="flex items-center justify-start">
        <span className="sr-only">Aura Digital Labs</span>
        <img
          src="/logo.png"
          width="60"
          height="40"
          alt="Hero"
          className="overflow-hidden rounded-t-xl object-cover"
        />
      </Link>
      <nav className="hidden md:flex gap-4 sm:gap-6 flex-1 justify-center">
        <Link
          href="#home"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Home
        </Link>
        <Link
          href="#about"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          About
        </Link>
        <Link
          href="#services"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Services
        </Link>
        <Link
          href="#projects"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Projects
        </Link>
        <Link
          href="#team"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Team
        </Link>
        <Link
          href="#contact"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Contact
        </Link>
      </nav>
      <a
        href="https://api.whatsapp.com/send/?phone=94714745349"
        target="_blank"
        rel="noopener noreferrer"
      >
        <Button
          className="hidden md:flex h-8 px-4 rounded-lg bg-primary/30 text-primary-foreground font-medium transition-colors hover:bg-primary/70 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
          style={{ borderRadius: "30px", overflow: "hidden" }}
        >
          Get Started
        </Button>
      </a>

      <div className="md:hidden flex items-center ml-4">
        <button
          id="menu-toggle"
          className="text-primary-foreground hover:text-primary-600 focus:outline-none"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            className="w-6 h-6"
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path
              strokeLinecap="round"
              strokeLinejoin="round"
              strokeWidth={2}
              d="M4 6h16M4 12h16m-7 6h7"
            />
          </svg>
        </button>
      </div>
      <div
        id="mobile-menu"
        className="absolute top-16 left-0 right-0 bg-white/80 shadow-lg rounded-lg p-4 flex flex-col items-center gap-4 z-40 hidden"
      >
        <Link
          href="#home"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Home
        </Link>
        <Link
          href="#about"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          About
        </Link>
        <Link
          href="#services"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Services
        </Link>
        <Link
          href="#projects"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Projects
        </Link>
        <Link
          href="#team"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Team
        </Link>
        <Link
          href="#contact"
          className="text-sm font-semibold hover:underline underline-offset-4"
          prefetch={false}
        >
          Contact
        </Link>
        <a
          href="https://api.whatsapp.com/send/?phone=94714745349"
          target="_blank"
          rel="noopener noreferrer"
        >
          <Button
            className="hidden md:flex h-8 px-4 rounded-lg bg-primary/30 text-primary-foreground font-medium transition-colors hover:bg-primary/70 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
            style={{ borderRadius: "30px", overflow: "hidden" }}
          >
            Get Started
          </Button>
        </a>
      </div>
    </header>
  );
}
