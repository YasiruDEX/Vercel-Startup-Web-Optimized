export default function Footer() {
    return (
      <footer className="w-full py-10 bg-background">
        <div className="container mx-auto flex flex-col items-center justify-between gap-4 px-4 text-center md:flex-row md:px-6">
          <p className="text-sm text-muted-foreground">
            © 2024 Aura Digital Labs. All rights reserved.
          </p>
          <nav className="flex gap-4 text-sm">
            <a href="#privacy-policy" className="hover:underline">Privacy Policy</a>
            <a href="#terms-of-service" className="hover:underline">Terms of Service</a>
          </nav>
        </div>
      </footer>
    );
  }
  