"use client";
import { useState, useEffect } from "react";
import Link from "next/link";
import { Post } from "@/app/blog/interfaces/post";
import { ArrowUpRight, Calendar, User } from "lucide-react";

export default function LatestNewsSection() {
  const [posts, setPosts] = useState<Post[]>([]);
  const [isInView, setIsInView] = useState(false);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    // Fetch posts from API route
    const fetchPosts = async () => {
      try {
        const response = await fetch('/api/latest-posts');
        if (response.ok) {
          const data = await response.json();
          setPosts(data);
        }
      } catch (error) {
        console.error("Error fetching posts:", error);
      } finally {
        setLoading(false);
      }
    };

    fetchPosts();

    // Intersection Observer for animations
    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsInView(true);
        }
      },
      { threshold: 0.1 }
    );

    const section = document.getElementById('latest-news');
    if (section) {
      observer.observe(section);
    }

    return () => {
      if (section) {
        observer.unobserve(section);
      }
    };
  }, []);

  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString("en-US", {
      year: "numeric",
      month: "long",
      day: "numeric",
    });
  };

  return (
    <section id="latest-news" className="w-full bg-white py-20 lg:py-32">
      <div className="max-w-7xl mx-auto px-6 lg:px-8">
        {/* Header */}
        <div
          className={`text-center mb-16 transition-all duration-700 ${
            isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
          }`}
        >
          <div className="flex items-center justify-center mb-4">
            <span className="text-blue-500 text-sm font-semibold tracking-wider uppercase">
              OUR BLOGS
            </span>
          </div>
          <h2 className="text-4xl lg:text-5xl xl:text-6xl font-bold text-gray-900 mb-6">
            LATEST <span className="text-gray-600">NEWS.</span>
          </h2>
          <div className="flex items-center justify-center">
            <Link
              href="/blog"
              className="group inline-flex items-center px-6 py-3 border border-gray-900 text-gray-900 font-medium rounded-full hover:bg-gray-900 hover:text-white transition-all duration-300"
            >
              VIEW ALL
              <ArrowUpRight className="ml-2 w-4 h-4 group-hover:translate-x-1 group-hover:-translate-y-1 transition-transform duration-300" />
            </Link>
          </div>
        </div>

        {/* Blog Posts Grid */}
        {loading ? (
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
            {[1, 2, 3].map((index) => (
              <div
                key={index}
                className="bg-white border border-gray-200 rounded-2xl overflow-hidden animate-pulse"
              >
                <div className="h-64 bg-gray-200"></div>
                <div className="p-6">
                  <div className="flex space-x-4 mb-4">
                    <div className="h-4 bg-gray-200 rounded w-20"></div>
                    <div className="h-4 bg-gray-200 rounded w-24"></div>
                  </div>
                  <div className="h-6 bg-gray-200 rounded mb-3"></div>
                  <div className="h-4 bg-gray-200 rounded mb-2"></div>
                  <div className="h-4 bg-gray-200 rounded w-3/4"></div>
                </div>
              </div>
            ))}
          </div>
        ) : posts.length === 0 ? (
          <div className="text-center py-12">
            <p className="text-gray-500 text-lg">No blog posts available at the moment.</p>
          </div>
        ) : (
          <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
            {posts.map((post, index) => (
              <article
                key={post.slug}
                className={`group bg-white border border-gray-200 rounded-2xl overflow-hidden hover:shadow-xl transition-all duration-500 hover:border-gray-300 hover:-translate-y-1 ${
                  isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
                }`}
                style={{ transitionDelay: `${index * 150}ms` }}
              >
                <Link href={`/blog/posts/${post.slug}`}>
                  {/* Image */}
                  <div className="relative h-64 overflow-hidden">
                    <img
                      src={post.coverImage}
                      alt={post.title}
                      className="w-full h-full object-cover group-hover:scale-105 transition-transform duration-700"
                    />
                    <div className="absolute inset-0 bg-gradient-to-t from-black/20 to-transparent opacity-0 group-hover:opacity-100 transition-opacity duration-300" />
                  </div>

                  {/* Content */}
                  <div className="p-6">
                    {/* Meta Information */}
                    <div className="flex items-center text-sm text-gray-500 mb-4 space-x-4">
                      <div className="flex items-center">
                        <User className="w-4 h-4 mr-1" />
                        <span>BY: {post.author.name.toUpperCase()}</span>
                      </div>
                      <div className="flex items-center">
                        <Calendar className="w-4 h-4 mr-1" />
                        <span>{formatDate(post.date).toUpperCase()}</span>
                      </div>
                    </div>

                    {/* Title */}
                    <h3 className="text-xl font-bold text-gray-900 mb-3 group-hover:text-blue-500 transition-colors duration-300 h-14 overflow-hidden" style={{
                      display: '-webkit-box',
                      WebkitLineClamp: 2,
                      WebkitBoxOrient: 'vertical'
                    }}>
                      {post.title}
                    </h3>

                    {/* Excerpt */}
                    <p className="text-gray-600 text-sm leading-relaxed h-16 overflow-hidden" style={{
                      display: '-webkit-box',
                      WebkitLineClamp: 3,
                      WebkitBoxOrient: 'vertical'
                    }}>
                      {post.excerpt}
                    </p>

                    {/* Read More Arrow */}
                    <div className="mt-4 flex items-center justify-end">
                      <div className="w-8 h-8 rounded-full border border-gray-300 flex items-center justify-center group-hover:border-blue-500 group-hover:bg-blue-500 transition-all duration-300">
                        <ArrowUpRight className="w-4 h-4 text-gray-600 group-hover:text-white transition-colors duration-300" />
                      </div>
                    </div>
                  </div>
                </Link>
              </article>
            ))}
          </div>
        )}

        {/* Bottom CTA */}
        <div
          className={`text-center mt-16 transition-all duration-700 delay-500 ${
            isInView ? "opacity-100 translate-y-0" : "opacity-0 translate-y-8"
          }`}
        >
          <p className="text-gray-600 mb-6 max-w-2xl mx-auto">
            Stay updated with the latest developments in AI, technology, and innovation. 
            Explore our comprehensive blog for in-depth analysis and industry insights.
          </p>
          <Link
            href="/blog"
            className="inline-flex items-center px-8 py-4 bg-gray-900 text-white font-semibold rounded-full hover:bg-gray-800 transition-all duration-300 hover:shadow-lg hover:scale-105"
          >
            Explore All Articles
            <ArrowUpRight className="ml-2 w-5 h-5" />
          </Link>
        </div>
      </div>
    </section>
  );
}
