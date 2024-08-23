// 'use client';
// import React, { useEffect, useState } from "react";
import { MainScreenLanding } from "@/components/MainScreenLanding";
import { SplashScreen } from "@/components/splash/splash";

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
      {/* {showSplash ? <SplashScreen /> : <MainScreenLanding />} */}
      <MainScreenLanding />
    </main>
  );
}
