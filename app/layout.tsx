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
      <body className={inter.className}>
        {children}
        {/* Start of Tawk.to Script */}
        <script type="text/javascript">
          {`
          var Tawk_API=Tawk_API||{}, Tawk_LoadStart=new Date();
          (function(){
          var s1=document.createElement("script"),s0=document.getElementsByTagName("script")[0];
          s1.async=true;
          s1.src='https://embed.tawk.to/66d18a44ea492f34bc0bad28/1i6h887rt';
          s1.charset='UTF-8';
          s1.setAttribute('crossorigin','*');
          s0.parentNode.insertBefore(s1,s0);
          })();
          `}
        </script>
        {/* End of Tawk.to Script */}
      </body>
    </html>
  );
}
