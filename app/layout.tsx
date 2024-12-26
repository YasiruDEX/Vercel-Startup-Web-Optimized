import { Inter } from "next/font/google";
import "./globals.css";
import SmoothScroll from "@/components/smooth-scroll";
import { CMS_NAME, HOME_OG_IMAGE_URL } from "@/app/blog/lib/constants";

const inter = Inter({ subsets: ["latin"], weight: ["400", "700"] });

export const metadata = {
  title: "Aura Digital Labs",
  description: "Unleash the Power of Digital Transformation",
  openGraph: {
    images: [HOME_OG_IMAGE_URL],
  },
};

import React, { ReactNode } from "react";

export default function RootLayout({ children }: { children: ReactNode }) {
  return (
    <html lang="en">
      <head>
        <link
          rel="aura"
          href="/assets/blog/authors/aura.png"
        />
        <link
          rel="icon"
          type="image/png"
          sizes="32x32"
          href="/assets/blog/authors/aura.png"
        />
        <link
          rel="icon"
          type="image/png"
          sizes="16x16"
          href="/assets/blog/authors/aura.png"
        />
        <link rel="manifest" href="/favicon/site.webmanifest" />
        <link
          rel="mask-icon"
          href="/assets/blog/authors/aura.png"
          color="#000000"
        />
        <link rel="shortcut icon" href="/assets/blog/authors/aura.png" />
        <meta name="msapplication-TileColor" content="#000000" />
        <meta
          name="msapplication-config"
          content="/favicon/browserconfig.xml"
        />
        <meta name="theme-color" content="#000" />
        <link rel="alternate" type="application/rss+xml" href="/feed.xml" />
      </head>
      <SmoothScroll />
      
      <body className={inter.className}>

        <script src="https://cdn.botpress.cloud/webchat/v2.2/inject.js"></script>
        <script src="https://files.bpcontent.cloud/2024/12/26/07/20241226071135-Q1L2LKIY.js"></script>
    
        {children}
        </body>
    </html>
  );
}
