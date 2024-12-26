'use client'

import { useState, useEffect } from 'react'
import Image from 'next/image'
import { ChevronLeft, ChevronRight } from 'lucide-react'

const images = [
  // '/Banner/banner 0 n.jpg',
  // '/Banner/banner 2.jpg',
  '/Banner/banner 3 n.jpg',
  '/Banner/banner 4.jpg',
  '/Banner/banner 8.jpg',
  '/Banner/banner 9.jpg',
  '/Banner/banner 7.jpg',
  '/Banner/banner 5 n.jpg',
  '/Banner/banner 6.jpg',
  '/Banner/banner 1.jpg',
]

export function BannerSlideshow() {
  const [currentSlide, setCurrentSlide] = useState(0)

  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentSlide((prevSlide) => (prevSlide + 1) % images.length)
    }, 5000)

    return () => clearInterval(timer)
  }, [])

  const goToSlide = (index: number) => {
    setCurrentSlide(index)
  }

  const goToPrevSlide = () => {
    setCurrentSlide((prevSlide) => (prevSlide - 1 + images.length) % images.length)
  }

  const goToNextSlide = () => {
    setCurrentSlide((prevSlide) => (prevSlide + 1) % images.length)
  }

  return (
    <div className="relative w-full px-0 shadow-md">
      <div
        className="flex transition-transform duration-500"
        style={{
          transform: `translateX(-${currentSlide * 100}vw)`,
        }}
      >
        {images.map((src, index) => (
          <div
            key={index}
            className="flex-none w-screen h-auto relative aspect-[16/8]"
          >
            <Image
              src={src}
              alt={`Slide ${index + 1}`}
              fill
              className="object-cover"
              priority={index === 0}
            />
          </div>
        ))}
      </div>

      {/* Navigation Arrows */}
      <button
        onClick={goToPrevSlide}
        className="absolute left-4 top-1/2 -translate-y-1/2 bg-black/50 text-white p-2 rounded-full hover:bg-black/75 transition-colors"
        aria-label="Previous slide"
      >
        <ChevronLeft className="w-12 h-12" />
      </button>
      <button
        onClick={goToNextSlide}
        className="absolute right-4 top-1/2 -translate-y-1/2 bg-black/50 text-white p-2 rounded-full hover:bg-black/75 transition-colors"
        aria-label="Next slide"
      >
        <ChevronRight className="w-12 h-12" />
      </button>
    </div>
  )
}

