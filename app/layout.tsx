import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import SmoothScroll from "@/components/smooth-scroll";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: "Aura Digital Labs",
  description: "Unleash the Power of Digital Transformation",
  icons: [
    {
      rel: "icon",
      url: "images/favicon.ico",
      href: "images/favicon.ico",
    },
  ],
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <SmoothScroll />
      <body className={inter.className}>{children}</body>
    </html>
  );
}
