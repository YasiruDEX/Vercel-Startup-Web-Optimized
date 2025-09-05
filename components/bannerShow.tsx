'use client'

import { useState, useEffect, useCallback } from 'react'
import Image from 'next/image'
import { ChevronLeft, ChevronRight } from 'lucide-react'

const images = [
  '/Banner/banner 3 n.jpg',
  '/Banner/banner 4.jpg',
  '/Banner/Banner 8.jpg',
  '/Banner/Banner 9.jpg',
  '/Banner/Banner 7.jpg',
  '/Banner/banner 6 n.jpg',
  '/Banner/banner 5 nn.jpg',
  '/Banner/banner 1.jpg',
]

const SLIDE_DURATION = 6000 // 6 seconds per slide

export function BannerSlideshow() {
  const [currentSlide, setCurrentSlide] = useState(0)
  const [progress, setProgress] = useState(0)

  const goToSlide = useCallback((index: number) => {
    setCurrentSlide(index)
    setProgress(0)
  }, [])

  const goToPrevSlide = useCallback(() => {
    setCurrentSlide((prevSlide) => (prevSlide - 1 + images.length) % images.length)
    setProgress(0)
  }, [])

  const goToNextSlide = useCallback(() => {
    setCurrentSlide((prevSlide) => (prevSlide + 1) % images.length)
    setProgress(0)
  }, [])

  // Auto-advance slides with progress tracking
  useEffect(() => {
    const progressInterval = setInterval(() => {
      setProgress((prev) => {
        if (prev >= 100) {
          setCurrentSlide((prevSlide) => (prevSlide + 1) % images.length)
          return 0
        }
        return prev + (100 / (SLIDE_DURATION / 50)) // Update every 50ms
      })
    }, 50)

    return () => clearInterval(progressInterval)
  }, [])

  // Reset progress when slide changes manually
  useEffect(() => {
    setProgress(0)
  }, [currentSlide])

  return (
    <div className="relative w-full overflow-hidden group">
      {/* Image Container */}
      <div className="relative w-full aspect-[16/8] bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-900">
        {images.map((src, index) => (
          <div
            key={index}
            className={`absolute inset-0 transition-opacity duration-1000 ease-in-out ${
              index === currentSlide ? 'opacity-100' : 'opacity-0'
            }`}
          >
            <Image
              src={src}
              alt={`Slide ${index + 1}`}
              fill
              className="object-cover"
              priority={index === 0}
            />
            {/* Subtle overlay for better text readability if needed */}
            <div className="absolute inset-0 bg-black/10" />
          </div>
        ))}
      </div>

      {/* Navigation Controls - Only visible on hover */}
      <div className="absolute inset-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300">
        <button
          onClick={goToPrevSlide}
          className="absolute left-6 top-1/2 -translate-y-1/2 bg-white/90 hover:bg-white text-gray-800 p-3 rounded-full shadow-lg hover:shadow-xl transition-all duration-200 backdrop-blur-sm"
          aria-label="Previous slide"
        >
          <ChevronLeft className="w-6 h-6" />
        </button>
        <button
          onClick={goToNextSlide}
          className="absolute right-6 top-1/2 -translate-y-1/2 bg-white/90 hover:bg-white text-gray-800 p-3 rounded-full shadow-lg hover:shadow-xl transition-all duration-200 backdrop-blur-sm"
          aria-label="Next slide"
        >
          <ChevronRight className="w-6 h-6" />
        </button>
      </div>

      {/* Progress Bar */}
      <div className="absolute bottom-0 left-0 right-0 bg-black/20 backdrop-blur-sm">
        <div className="flex items-center justify-center px-6 py-4">
          {/* Slide Indicators - Centered */}
          <div className="flex items-center space-x-3">
            {images.map((_, index) => (
              <button
                key={index}
                onClick={() => goToSlide(index)}
                className={`w-2 h-2 rounded-full transition-all duration-300 ${
                  index === currentSlide
                    ? 'bg-white scale-125'
                    : 'bg-white/40 hover:bg-white/60'
                }`}
                aria-label={`Go to slide ${index + 1}`}
              />
            ))}
          </div>
        </div>

        {/* Progress Bar */}
        <div className="h-1 bg-white/20">
          <div 
            className="h-full bg-white transition-all duration-75 ease-linear"
            style={{ width: `${progress}%` }}
          />
        </div>
      </div>

      {/* Slide Counter */}
      <div className="absolute top-6 right-6 bg-black/50 text-white px-3 py-1 rounded-full text-sm font-medium backdrop-blur-sm">
        {currentSlide + 1} / {images.length}
      </div>
    </div>
  )
}