"use client";

import { useEffect, useRef, useState } from "react";

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
  const popupRef = useRef<HTMLDivElement | null>(null);

  const [hovered, setHovered] = useState(false);
  const [currentImage, setCurrentImage] = useState(image1);
  const [currentIndex, setCurrentIndex] = useState(0);

  const images = [image1, image2, image3];

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (popupRef.current && !popupRef.current.contains(event.target as Node)) {
        setShowPopup(false);
      }
    };

    document.addEventListener("mousedown", handleClickOutside);
    return () => {
      document.removeEventListener("mousedown", handleClickOutside);
    };
  }, []);

  useEffect(() => {
    // Update the image index every 5 seconds
    const interval = setInterval(() => {
      setCurrentIndex((prevIndex) => (prevIndex + 1) % images.length);
    }, 1000); // 1 seconds

    return () => clearInterval(interval);
  }, [images.length]);

  useEffect(() => {
    setCurrentImage(images[currentIndex]);
  }, [currentIndex, images]);

  return (
    <div
      className="group relative overflow-hidden rounded-lg"
      onClick={() => setShowPopup(true)}
    >
      <div className="relative w-full h-full">
        <div
          className="group relative overflow-hidden rounded-lg"
          onMouseEnter={() => setHovered(true)}
          onMouseLeave={() => {
            setHovered(false);
            setCurrentImage(image1); // Reset to first image when not hovered
          }}
        >
          <div className="relative w-full h-full">
            <img
              src={currentImage}
              width={400}
              height={400}
              alt="Project Image"
              className="rounded-md object-cover w-full h-full transition-transform duration-300 ease-in-out transform hover:scale-105"
              style={{ aspectRatio: "1 / 1" }}
            />
          </div>

          <div className="absolute bottom-0 left-0 right-0 bg-black/70 p-4 text-white opacity-0 transition-opacity duration-300 ease-in-out group-hover:opacity-100 bg-background">
            <h3 className="text-lg font-semibold">{title}</h3>
            <p className="text-sm">{description}</p>
          </div>
        </div>
      </div>

      {showPopup && (
        <div className="fixed inset-0 flex items-center justify-center bg-black/70 p-6 z-50 transition-opacity duration-500 ease-in-out opacity-100 overflow-visible">
          <div
            ref={popupRef}
            className="bg-background p-8 rounded-lg shadow-2xl text-black max-w-5xl w-full relative h-[80vh] transition-transform duration-500 ease-in-out transform scale-105 animate-popup popup-content overflow-visible"
          >
            <button
              className="absolute top-4 right-4 text-3xl text-gray-800"
              onClick={(e) => {
                e.stopPropagation(); // Prevents click from propagating to the parent div
                setShowPopup(false);
              }}
            >
              &times;
            </button>

            <h3 className="text-4xl font-bold mb-4 text-gray-900">{title}</h3>
            <p className="text-xl mb-4 text-gray-800">{description}</p>
            {source && (
              <p className="text-xl mb-6 text-gray-800">
                Source:{" "}
                <a
                  href={source}
                  className="text-blue-600 underline"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  View Video
                </a>
              </p>
            )}

            <div className="mb-6">
              <h2 className="text-2xl font-semibold mb-2">Technologies</h2>
              {technologies}
            </div>

            <div className="mb-6">
              <h2 className="text-2xl font-semibold mb-2">Description</h2>
              <p>
                {more_details}
              </p>
            </div>

            <div className="grid grid-cols-2 gap-6 sm:grid-cols-3 md:grid-cols-4">
              <img
                src={image1}
                width={300}
                height={200}
                alt="Landscape 1"
                className="rounded-md object-cover aspect-[3/2] transition-transform duration-300 ease-in-out transform hover:scale-105"
              />
              <img
                src={image2}
                width={300}
                height={200}
                alt="Landscape 2"
                className="rounded-md object-cover aspect-[3/2] transition-transform duration-300 ease-in-out transform hover:scale-105"
              />
              <img
                src={image3}
                width={300}
                height={200}
                alt="Landscape 3"
                className="rounded-md object-cover aspect-[3/2] transition-transform duration-300 ease-in-out transform hover:scale-105"
              />
            </div>
          </div>
        </div>
      )}

      <style jsx>{`
        @keyframes popupIn {
          from {
            opacity: 0;
            transform: scale(0.9);
          }
          to {
            opacity: 1;
            transform: scale(1);
          }
        }

        @keyframes popupOut {
          from {
            opacity: 1;
            transform: scale(1);
          }
          to {
            opacity: 0;
            transform: scale(0.9);
          }
        }

        .animate-popup {
          animation: popupIn 0.5s ease-out;
        }

        .animate-popup-exit {
          animation: popupOut 0.5s ease-in;
        }

        /* Hide scrollbar but keep the content scrollable */
        .popup-content {
          overflow: auto; /* Ensure content is scrollable */
        }

        .popup-content::-webkit-scrollbar {
          display: none; /* Hide scrollbar for Webkit browsers (Chrome, Safari) */
        }

        .popup-content {
          scrollbar-width: none; /* Hide scrollbar for Firefox */
          -ms-overflow-style: none; /* Hide scrollbar for Internet Explorer and Edge */
        }
      `}</style>
    </div>
  );
}
