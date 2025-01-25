import { Button } from "@/components/ui/button"
import Link from "next/link";
import { Cover } from "@/components/ui/cover";
import {
  FaAngleDown,
} from "react-icons/fa";

export default function AboutSection() {
  return (
    <div>
      <div className="flex justify-center items-center h-20 animate-up-down">
        <FaAngleDown className="text-4xl text-slate-900" />
      </div>

      <style jsx>{`
        .animate-up-down {
          animation: upDown 2s infinite ease-in-out;
        }

        @keyframes upDown {
          0%, 100% {
            transform: translateY(0);
          }
          50% {
            transform: translateY(10px); /* Adjust the distance to your liking */
          }
        }
      `}</style>
      <div className="flex flex-col items-center justify-center min-h-[80vh] px-4 py-20 text-center max-w-6xl mx-auto">
        {/* <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
          <span className="text-slate-900">Transforming Digital Dreams into</span>{' '}
          <span className="bg-gradient-to-r from-gray-400 to-black bg-clip-text text-transparent font-bold">
            Powerful Reality
          </span>      
        </h1> */}

      <h1 className="text-4xl md:text-4xl lg:text-6xl font-semibold max-w-7xl mx-auto text-center mt-6 relative z-20 py-6 bg-clip-text text-transparent bg-gradient-to-b from-neutral-800 via-neutral-700 to-neutral-300 dark:from-neutral-80">
      Transforming Digital Dreams <br /> into <Cover>Powerful Reality</Cover>
      </h1>
        
        <p className="text-slate-600 font-thin text-sm md:text-md max-w-3xl mb-12">
          We specialize in turning your digital vision into reality through innovative solutions, 
          cutting-edge technology, and expert implementation. Let us guide your journey from concept to success.
        </p>

        <div 
        className="flex flex-col sm:flex-row items-center gap-4 bg-gray-400 p-6 bg-opacity-10 py-6"
        style={{
          objectFit: "cover",
          borderRadius: "30px",
        }}>
          <div className="text-slate-900 font-semibold text-md">
            Ready to Transform Your Digital Presence?
          </div>
          <a
            href="https://api.whatsapp.com/send?phone=94754745359&text=Hi!%20I%27d%20like%20to%20connect%20with%20the%20Aura%20Digital%20Labs%20team%20for%20a%20discussion."
            target="_blank"
            rel="noopener noreferrer"
          >
          <Button 
            size="lg"
            className="bg-gradient-to-r from-gray-700 to-black hover:from-gray-500 hover:to-gray-700 text-white font-medium p-4 rounded-lg"
            style={{
                objectFit: "cover",
                borderRadius: "20px",
            }}
          >
            Let&apos;s Discuss
          </Button>
          </a>
        </div>
      </div>
    </div>
  )
}

