import Footer from "@/app/blog/_components/footer";
import { CMS_NAME, HOME_OG_IMAGE_URL } from "@/app/blog/lib/constants";
import type { Metadata } from "next";
import { Inter } from "next/font/google";
import cn from "classnames";
// import { ThemeSwitcher } from "./_components/theme-switcher";

import "./globals.css";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: `Blog by AURA Digital Labs`,
  description: `Blog by AURA Digital Labs.`,
  openGraph: {
    images: [HOME_OG_IMAGE_URL],
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
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
        <meta name="theme-color" content="#ffffff" />
        <link rel="alternate" type="application/rss+xml" href="/feed.xml" />
      </head>
      <body className={cn(inter.className, "bg-white text-black")}>
        {/* <ThemeSwitcher /> */}
        <div className="min-h-screen">{children}</div>
        <Footer />
      </body>
    </html>
  );
}