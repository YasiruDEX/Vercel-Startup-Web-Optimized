import Link from "next/link";
import { useDarkMode } from "@/components/darkModeProvider";

export default function FooterSection() {
  const { darkMode } = useDarkMode();

  return (
    <footer className={`${darkMode ? "" : "bg-muted"} py-12`}>
      <div className="container items-center justify-center gap-4 px-4 text-center md:px-6 mx-auto grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
        <div className="space-y-4">
          <h3 className="text-lg font-semibold">Want to know more?</h3>
          <p className="text-muted-foreground">
            Book a 1-to-1 meeting with our team to discuss your project.
          </p>
          <Link
            href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27d%20like%20to%20connect%20with%20the%20Aura%20Digital%20Labs%20team%20for%20a%20discussion."
            className="inline-flex items-center justify-center rounded-md bg-primary px-4 py-2 text-sm font-medium text-primary-foreground shadow transition-colors hover:bg-primary/90 focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
            prefetch={false}
          >
            Book a Meeting
          </Link>
        </div>
        <div className="space-y-4">
          <h3 className="text-lg font-semibold">About Aura Digital Labs</h3>
          <p className="text-muted-foreground">
            Learn more about our company and our mission.
          </p>
          <Link
            href="https://www.auradigitallabs.com"
            className="text-sm text-muted-foreground hover:underline underline-offset-4"
            prefetch={false}
          >
            About Us
          </Link>
        </div>
        <div className="space-y-4">
          <h3 className="text-lg font-semibold">Terms and Conditions</h3>
          <p className="text-muted-foreground">
            Review our terms and conditions for using our services.
          </p>
          <Link
            href="#"
            className="text-sm text-muted-foreground hover:underline underline-offset-4"
            prefetch={false}
          >
            Terms of Service
          </Link>
        </div>
        <div className="space-y-4">
          <h3 className="text-lg font-semibold">Privacy Policy</h3>
          <p className="text-muted-foreground">
            Learn how we protect your personal information.
          </p>
          <Link
            href="#"
            className="text-sm text-muted-foreground hover:underline underline-offset-4"
            prefetch={false}
          >
            Privacy Policy
          </Link>
        </div>
      </div>
      <div className="container mt-12 flex flex-col items-center justify-center px-10 text-center md:px-10 mx-auto gap-4 md:flex-row md:justify-between">
        <div className="flex items-center gap-4">
          <Link
            href="https://www.facebook.com/reel/1159289995132371"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <FacebookIcon className="h-5 w-5" />
            <span className="sr-only">Facebook</span>
          </Link>
          {/* <Link
            href="https://twitter.com/auradigitallabs"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <TwitterIcon className="h-5 w-5" />
            <span className="sr-only">Twitter</span>
          </Link> */}
          <Link
            href="https://www.linkedin.com/feed/update/urn:li:activity:7233808706751053824"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <LinkedinIcon className="h-5 w-5" />
            <span className="sr-only">LinkedIn</span>
          </Link>
          {/* <Link
            href="https://www.instagram.com/auradigitallabs"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <InstagramIcon className="h-5 w-5" />
            <span className="sr-only">Instagram</span>
          </Link> */}
          <Link
            href="https://youtube.com/shorts/WaVciSVUl00?si=3n80yKFPBPBeTFdy"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <YouTubeIcon className="h-5 w-5" />
            <span className="sr-only">YouTube</span>
          </Link>
          <Link
            href="https://vt.tiktok.com/ZS2rKYWTV/"
            className="text-muted-foreground hover:text-muted"
            prefetch={false}
          >
            <TikTokIcon className="h-5 w-5" />
            <span className="sr-only">TikTok</span>
          </Link>
        </div>
        <p className="text-xs text-muted-foreground">
          &copy; 2024 Aura Digital Labs. All rights reserved.
        </p>
      </div>
    </footer>
  );
}

function FacebookIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M18 2h-3a5 5 0 0 0-5 5v3H7v4h3v8h4v-8h3l1-4h-4V7a1 1 0 0 1 1-1h3z" />
    </svg>
  );
}

function InstagramIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <rect width="20" height="20" x="2" y="2" rx="5" ry="5" />
      <path d="M16 11.37A4 4 0 1 1 12.63 8 4 4 0 0 1 16 11.37z" />
      <line x1="17.5" x2="17.51" y1="6.5" y2="6.5" />
    </svg>
  );
}

function LinkedinIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M16 8a6 6 0 0 1 6 6v7h-4v-7a2 2 0 0 0-2-2 2 2 0 0 0-2 2v7h-4v-7a6 6 0 0 1 6-6z" />
      <rect width="4" height="12" x="2" y="9" />
      <circle cx="4" cy="4" r="2" />
    </svg>
  );
}

function TwitterIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M22 4s-.7 2.1-2 3.4c1.6 10-9.4 17.3-18 11.6 2.2.1 4.4-.6 6-2C3 15.5.5 9.6 3 5c2.2 2.6 5.6 4.1 9 4-.9-4.2 4-6.6 7-3.8 1.1 0 3-1.2 3-1.2z" />
    </svg>
  );
}

function YouTubeIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M22.54 6.42A2.78 2.78 0 0 0 20.8 4.7C18.87 4 12 4 12 4s-6.87 0-8.8.7A2.78 2.78 0 0 0 1.46 6.42a29.94 29.94 0 0 0-.46 5.58 29.94 29.94 0 0 0 .46 5.58 2.78 2.78 0 0 0 1.74 1.72c1.93.7 8.8.7 8.8.7s6.87 0 8.8-.7a2.78 2.78 0 0 0 1.74-1.72 29.94 29.94 0 0 0 .46-5.58 29.94 29.94 0 0 0-.46-5.58z" />
      <polygon points="9.75 15.02 15.56 12 9.75 8.98 9.75 15.02" />
    </svg>
  );
}

function TikTokIcon(props: any) {
  return (
    <svg
      {...props}
      xmlns="http://www.w3.org/2000/svg"
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <path d="M9 8h1.8a5.4 5.4 0 0 0 2.7-5H9z" />
      <path d="M18.7 5.5A6.6 6.6 0 0 1 13.5 3V15a4.5 4.5 0 1 1-3.5-4.4" />
    </svg>
  );
}
