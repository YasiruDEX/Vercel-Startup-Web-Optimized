import { getAllPosts } from "@/app/blog/lib/api";
import { NextResponse } from "next/server";

export async function GET() {
  try {
    const allPosts = getAllPosts();
    const latestPosts = allPosts.slice(0, 3); // Get latest 3 posts
    
    return NextResponse.json(latestPosts);
  } catch (error) {
    console.error("Error fetching posts:", error);
    return NextResponse.json([], { status: 500 });
  }
}
