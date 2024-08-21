import Link from "next/link";

export default function Header() {
  return (
    <header className="px-4 lg:px-6 h-14 flex items-center">
      <Link href="#" className="flex items-center justify-center font-bold text-lg lg:text-xl">
        <span className="sr-only">Aura Digital Labs</span>
        Aura Digital Labs
      </Link>
      <nav className="ml-auto flex gap-4 sm:gap-6">
        <Link href="#home" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>Home</Link>
        <Link href="#about" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>About</Link>
        <Link href="#services" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>Services</Link>
        <Link href="#team" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>Team</Link>
        <Link href="#contact" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>Contact</Link>
        <Link href="#projects" className="text-sm font-medium hover:underline underline-offset-4" prefetch={false}>Projects</Link>
      </nav>
    </header>
  );
}
