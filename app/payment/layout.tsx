import { Inter } from "next/font/google";
import "./globals.css";
import SmoothScroll from "@/components/smooth-scroll";

const inter = Inter({ subsets: ["latin"], weight: ["400", "700"] });

export const metadata = {
  title: "Aura Digital Labs",
  description: "Unleash the Power of Digital Transformation",
  icons: {
    rel: "icon",
    href: "/images/favicon.ico",
  },
};

import React, { ReactNode } from "react";

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="en">
      <SmoothScroll />
      
      <body className={inter.className}>{children}</body>
    </html>
  );
}
