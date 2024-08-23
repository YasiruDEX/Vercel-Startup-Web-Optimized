import { type ClassValue, clsx } from "clsx"
import { twMerge } from "tailwind-merge"
// styles/font.ts
import { Poppins } from "next/font/google";

const poppins = Poppins({
  subsets: ['latin'],
  weight: ['400', '700'], // Specify the font weights you want to include
});

export default poppins;


export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}
