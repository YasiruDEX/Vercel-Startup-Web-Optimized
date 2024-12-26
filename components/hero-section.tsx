import { Button } from "@/components/ui/button"

export default function HeroSection() {
  return (
    <div className="flex flex-col items-center justify-center min-h-[80vh] px-4 py-20 text-center max-w-6xl mx-auto">
      <h1 className="text-4xl md:text-6xl font-bold tracking-tight mb-6">
        <span className="text-slate-900">Transforming Digital Dreams into</span>{' '}
        <span className="text-orange-500">Powerful Reality</span>
      </h1>
      
      <p className="text-slate-600 text-lg md:text-xl max-w-3xl mb-12">
        We specialize in turning your digital vision into reality through innovative solutions, 
        cutting-edge technology, and expert implementation. Let us guide your journey from concept to success.
      </p>

      <div className="flex flex-col sm:flex-row items-center gap-4 bg-slate-50 p-6 rounded-lg">
        <div className="text-slate-900 font-medium">
          Ready to Transform Your Digital Presence?
        </div>
        <Button 
          size="lg"
          className="bg-blue-600 hover:bg-blue-700 text-white font-medium"
        >
          Start Your Project
        </Button>
      </div>
    </div>
  )
}

