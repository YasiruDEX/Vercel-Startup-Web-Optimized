"use client";

import { useEffect, useRef, useState } from "react";
import { motion, AnimatePresence } from "framer-motion";
import { X, ExternalLink, Eye } from "lucide-react";
import { Button } from "@/components/ui/button";

interface ProjectsProps {
  description: string;
  title: string;
  source?: string;
  more_details?: string;
  technologies?: string;
  image1: string;
  image2: string;
  image3: string;
}

export default function Projects({
  description,
  title,
  source,
  image1,
  image2,
  image3,
  more_details,
  technologies,
}: ProjectsProps) {
  const [showPopup, setShowPopup] = useState(false);
  const [currentImage, setCurrentImage] = useState(image1);
  const [currentIndex, setCurrentIndex] = useState(0);
  const [isHovered, setIsHovered] = useState(false);

  const images = [image1, image2, image3];

  useEffect(() => {
    // Auto-switch images every 3 seconds
    const interval = setInterval(() => {
      setCurrentIndex((prevIndex) => (prevIndex + 1) % images.length);
    }, 3000);

    return () => clearInterval(interval);
  }, [images.length]);

  useEffect(() => {
    setCurrentImage(images[currentIndex]);
  }, [currentIndex, images]);

  const handleKeyDown = (e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      setShowPopup(false);
    }
  };

  useEffect(() => {
    if (showPopup) {
      document.addEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'hidden';
    } else {
      document.removeEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'unset';
    }

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'unset';
    };
  }, [showPopup]);

  return (
    <>
      <motion.div
        className="group relative overflow-hidden rounded-2xl cursor-pointer bg-card/50 backdrop-blur-sm border border-border/50"
        onClick={() => setShowPopup(true)}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        whileHover={{ y: -8 }}
        transition={{ duration: 0.3, ease: "easeOut" }}
      >
        {/* Image Container */}
        <div className="relative w-full aspect-square overflow-hidden">
          <motion.img
            src={currentImage}
            alt="Project Image"
            className="w-full h-full object-cover transition-all duration-500"
            initial={{ scale: 1 }}
            whileHover={{ scale: 1.05 }}
            transition={{ duration: 0.5 }}
          />
          
          {/* Enhanced Gradient Overlay for better text readability */}
          <div className="absolute inset-0 bg-gradient-to-t from-black/90 via-black/30 to-transparent opacity-70 group-hover:opacity-50 transition-opacity duration-300" />
          
          {/* Hover Overlay with View Button */}
          <motion.div 
            className="absolute inset-0 bg-black/20 backdrop-blur-[1px] flex items-center justify-center opacity-0 group-hover:opacity-100 transition-all duration-300"
            initial={{ opacity: 0 }}
            whileHover={{ opacity: 1 }}
          >
            <motion.div
              className="bg-white/90 backdrop-blur-sm text-black px-6 py-3 rounded-full font-medium flex items-center gap-2 shadow-lg"
              initial={{ scale: 0, opacity: 0 }}
              animate={{ 
                scale: isHovered ? 1 : 0, 
                opacity: isHovered ? 1 : 0 
              }}
              transition={{ 
                duration: 0.3, 
                delay: isHovered ? 0.1 : 0,
                ease: "easeOut"
              }}
            >
              <Eye className="w-4 h-4" />
              View Details
            </motion.div>
          </motion.div>
        </div>

        {/* Content Overlay - Shows on hover with white background */}
        <div className="absolute bottom-0 left-0 right-0 opacity-0 group-hover:opacity-100 transition-opacity duration-300">
          {/* White/blur background for text area - only on hover */}
          <div className="bg-white/95 backdrop-blur-md border-t border-white/20 p-6">
            <motion.h3 
              className="text-xl font-bold mb-2 leading-tight text-black"
              initial={{ y: 10, opacity: 0.8 }}
              whileHover={{ y: 0, opacity: 1 }}
              transition={{ duration: 0.2 }}
            >
              {title}
            </motion.h3>
            <motion.p 
              className="text-sm text-black/80 line-clamp-2 leading-relaxed"
              initial={{ y: 10, opacity: 0.7 }}
              whileHover={{ y: 0, opacity: 1 }}
              transition={{ duration: 0.2, delay: 0.05 }}
            >
              {description}
            </motion.p>
          </div>
        </div>

        {/* Default text overlay - visible by default with gradient background */}
        <div className="absolute bottom-0 left-0 right-0 p-6 text-white group-hover:opacity-0 transition-opacity duration-300">
          <h3 className="text-xl font-bold mb-2 leading-tight">
            {title}
          </h3>
          <p className="text-sm text-white/90 line-clamp-2 leading-relaxed">
            {description}
          </p>
        </div>

        {/* Progress Dots for Image Slideshow */}
        <div className="absolute top-4 right-4 flex space-x-1">
          {images.map((_, index) => (
            <div
              key={index}
              className={`w-2 h-2 rounded-full transition-all duration-300 ${
                index === currentIndex ? 'bg-white scale-110' : 'bg-white/50'
              }`}
            />
          ))}
        </div>
      </motion.div>

      {/* Enhanced Modal */}
      <AnimatePresence>
        {showPopup && (
          <motion.div
            className="fixed inset-0 z-50 flex items-center justify-center p-4"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.3 }}
          >
            {/* Backdrop */}
            <motion.div
              className="absolute inset-0 bg-black/80 backdrop-blur-md"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              onClick={() => setShowPopup(false)}
            />

            {/* Modal Content */}
            <motion.div
              className="relative bg-background rounded-3xl shadow-2xl max-w-5xl w-full max-h-[90vh] overflow-hidden border border-border/50"
              initial={{ opacity: 0, scale: 0.8, y: 50 }}
              animate={{ opacity: 1, scale: 1, y: 0 }}
              exit={{ opacity: 0, scale: 0.8, y: 50 }}
              transition={{ 
                duration: 0.4, 
                ease: [0.25, 0.46, 0.45, 0.94]
              }}
            >
              {/* Header */}
              <div className="sticky top-0 bg-background/80 backdrop-blur-lg border-b border-border/50 p-6 flex justify-between items-start z-10">
                <div className="flex-1 pr-4">
                  <h2 className="text-3xl font-bold text-foreground mb-2">{title}</h2>
                  <p className="text-muted-foreground text-lg">{description}</p>
                </div>
                <motion.button
                  className="flex-shrink-0 w-10 h-10 bg-muted hover:bg-muted/80 rounded-full flex items-center justify-center transition-colors"
                  onClick={() => setShowPopup(false)}
                  whileHover={{ scale: 1.1 }}
                  whileTap={{ scale: 0.9 }}
                >
                  <X className="w-5 h-5" />
                </motion.button>
              </div>

              {/* Scrollable Content */}
              <div className="overflow-y-auto max-h-[calc(90vh-120px)] p-6 space-y-8">
                {/* Source Link */}
                {source && (
                  <motion.div
                    className="flex items-center gap-2 p-4 bg-primary/5 border border-primary/20 rounded-xl"
                    initial={{ opacity: 0, x: -20 }}
                    animate={{ opacity: 1, x: 0 }}
                    transition={{ delay: 0.1 }}
                  >
                    <ExternalLink className="w-5 h-5 text-primary" />
                    <span className="text-foreground font-medium">Demo:</span>
                    <a
                      href={source}
                      className="text-primary hover:text-primary/80 font-medium underline underline-offset-2 transition-colors"
                      target="_blank"
                      rel="noopener noreferrer"
                    >
                      View Video
                    </a>
                  </motion.div>
                )}

                {/* Technologies */}
                <motion.div
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.2 }}
                >
                  <h3 className="text-xl font-semibold text-foreground mb-4 flex items-center gap-2">
                    Technologies Used
                  </h3>
                  <div className="p-4 bg-muted/50 rounded-xl border border-border/50">
                    <p className="text-foreground leading-relaxed">{technologies}</p>
                  </div>
                </motion.div>

                {/* Description */}
                <motion.div
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.3 }}
                >
                  <h3 className="text-xl font-semibold text-foreground mb-4 flex items-center gap-2">
                    Project Details
                  </h3>
                  <div className="p-4 bg-muted/50 rounded-xl border border-border/50">
                    <p className="text-foreground leading-relaxed">{more_details}</p>
                  </div>
                </motion.div>

                {/* Image Gallery */}
                <motion.div
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.4 }}
                >
                  <h3 className="text-xl font-semibold text-foreground mb-4 flex items-center gap-2">
                    Project Gallery
                  </h3>
                  <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-6">
                    {images.map((image, index) => (
                      <motion.div
                        key={index}
                        className="relative group overflow-hidden rounded-xl border border-border/50"
                        initial={{ opacity: 0, scale: 0.8 }}
                        animate={{ opacity: 1, scale: 1 }}
                        transition={{ delay: 0.5 + index * 0.1 }}
                        whileHover={{ scale: 1.02 }}
                      >
                        <img
                          src={image}
                          alt={`Project image ${index + 1}`}
                          className="w-full h-48 object-cover transition-transform duration-300 group-hover:scale-105"
                        />
                        <div className="absolute inset-0 bg-black/0 group-hover:bg-black/10 transition-colors duration-300" />
                      </motion.div>
                    ))}
                  </div>
                </motion.div>
              </div>
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
}
