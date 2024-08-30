'use client';
import React, { useEffect } from "react";
import { MainScreenLanding } from "@/components/MainScreenLanding";
import { SplashScreen } from "@/components/splash/splash";
import { DarkModeProvider } from "@/components/darkModeProvider"; // Adjust path as necessary

export default function Home() {
  useEffect(() => {
    // Create the script element
    const script = document.createElement("script");
    script.src = "https://embed.tawk.to/66d18a44ea492f34bc0bad28/1i6h887rt";
    script.async = true;
    script.charset = "UTF-8";
    script.setAttribute("crossorigin", "*");

    // Append the script to the body
    document.body.appendChild(script);

    // Cleanup script on component unmount
    return () => {
      document.body.removeChild(script);
    };
  }, []);

  return (
    <main className="min-h-screen">
      <DarkModeProvider>
        {/* {showSplash ? <SplashScreen /> : <MainScreenLanding />} */}
        <MainScreenLanding />
      </DarkModeProvider>
    </main>
  );
}
