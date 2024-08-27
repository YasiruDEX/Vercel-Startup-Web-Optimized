// 'use client';
// import React, { useEffect, useState } from "react";
import { MainScreenLanding } from "@/components/MainScreenLanding";
import { SplashScreen } from "@/components/splash/splash";
import { DarkModeProvider } from "@/components/darkModeProvider"; // Adjust path as necessary

export default function Home() {
  // const [showSplash, setShowSplash] = useState(true);

  // useEffect(() => {
  //   const timer = setTimeout(() => {
  //     setShowSplash(false);
  //   }, 2000);

  //   // Cleanup the timeout if the component is unmounted
  //   return () => clearTimeout(timer);
  // }, []);

  return (
    <main className="min-h-screen">
      <DarkModeProvider>
      {/* {showSplash ? <SplashScreen /> : <MainScreenLanding />} */}
      <MainScreenLanding />
      </DarkModeProvider>
    </main>
  );
}
