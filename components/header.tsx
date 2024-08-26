import Link from "next/link";
import { Button } from "@/components/ui/button";
import { useState, useEffect } from "react";
import { styled } from "@mui/material/styles";
import FormControlLabel from "@mui/material/FormControlLabel";
import Switch from "@mui/material/Switch";

// Define the customized switch
const MaterialUISwitch = styled(Switch)(({ theme }) => ({
  width: 62,
  height: 34,
  padding: 7,
  "& .MuiSwitch-switchBase": {
    margin: 1,
    padding: 0,
    transform: "translateX(6px)",
    "&.Mui-checked": {
      color: "#fff",
      transform: "translateX(22px)",
      "& .MuiSwitch-thumb:before": {
        backgroundImage: `url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" height="20" width="20" viewBox="0 0 20 20"><path fill="${encodeURIComponent(
          "#fff"
        )}" d="M4.2 2.5l-.7 1.8-1.8.7 1.8.7.7 1.8.6-1.8L6.7 5l-1.9-.7-.6-1.8zm15 8.3a6.7 6.7 0 11-6.6-6.6 5.8 5.8 0 006.6 6.6z"/></svg>')`,
      },
      "& + .MuiSwitch-track": {
        opacity: 1,
        backgroundColor: theme.palette.mode === "dark" ? "#8796A5" : "#aab4be",
      },
    },
  },
  "& .MuiSwitch-thumb": {
    backgroundColor: theme.palette.mode === "dark" ? "#003892" : "#001e3c",
    width: 32,
    height: 32,
    "&::before": {
      content: "''",
      position: "absolute",
      width: "100%",
      height: "100%",
      left: 0,
      top: 0,
      backgroundRepeat: "no-repeat",
      backgroundPosition: "center",
      backgroundImage: `url('data:image/svg+xml;utf8,<svg xmlns="http://www.w3.org/2000/svg" height="20" width="20" viewBox="0 0 20 20"><path fill="${encodeURIComponent(
        "#fff"
      )}" d="M9.305 1.667V3.75h1.389V1.667h-1.39zm-4.707 1.95l-.982.982L5.09 6.072l.982-.982-1.473-1.473zm10.802 0L13.927 5.09l.982.982 1.473-1.473-.982-.982zM10 5.139a4.872 4.872 0 00-4.862 4.86A4.872 4.872 0 0010 14.862 4.872 4.872 0 0014.86 10 4.872 4.872 0 0010 5.139zm0 1.389A3.462 3.462 0 0113.471 10a3.462 3.462 0 01-3.473 3.472A3.462 3.462 0 016.527 10 3.462 3.462 0 0110 6.528zM1.665 9.305v1.39h2.083v-1.39H1.666zm14.583 0v1.39h2.084v-1.39h-2.084zM5.09 13.928L3.616 15.4l.982.982 1.473-1.473-.982-.982zm9.82 0l-.982.982 1.473 1.473.982-.982-1.473-1.473zM9.305 16.25v2.083h1.389V16.25h-1.39z"/></svg>')`,
    },
  },
  "& .MuiSwitch-track": {
    opacity: 1,
    backgroundColor: theme.palette.mode === "dark" ? "#8796A5" : "#aab4be",
    borderRadius: 20 / 2,
  },
}));

export default function HeaderSection() {
  const [darkMode, setDarkMode] = useState(false);

  useEffect(() => {
    document.documentElement.classList.toggle("dark", darkMode);
  }, [darkMode]);

  return (
    <header className="fixed top-0 left-0 right-0 px-4 lg:px-6 h-16 flex items-center justify-between backdrop-blur-md bg-white/20 shadow-sm z-50">
      <Link href="#" className="flex items-center justify-start">
        <span className="sr-only">Aura Digital Labs</span>
        <img
          src={darkMode ? "/logo_light.png" : "/logo.png"}
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

      <div className="fixed top-4 right-4 z-10 flex items-center space-x-4">
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

        <FormControlLabel
          control={
            <MaterialUISwitch
              checked={darkMode}
              onChange={() => setDarkMode(!darkMode)}
            />
          }
          label={darkMode ? "" : ""}
        />
      </div>

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
