import { getAllPosts } from "@/app/blog/lib/api";
import LatestNewsSection from "./latestNewsSection";

export default function LatestNewsSectionWrapper() {
  // This is a server component, so we can safely use getAllPosts here
  const allPosts = getAllPosts();
  const latestPosts = allPosts.slice(0, 3); // Get latest 3 posts

  return <LatestNewsSection posts={latestPosts} />;
}
