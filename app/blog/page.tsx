import Container from "@/app/blog/_components/container";
import { HeroPost } from "@/app/blog/_components/hero-post";
import { Intro } from "@/app/blog/_components/intro";
import { MoreStories } from "@/app/blog/_components/more-stories";
import { getAllPosts } from "@/app/blog/lib/api";

export default function Index() {
  const allPosts = getAllPosts();

  const heroPost = allPosts[0];

  const morePosts = allPosts.slice(1);

  return (
    // bigger screen mx is 20 and in smaller screen mx is 4 medium screen mx is 8
    <main className="mx-4 sm:mx-20 md:mx-8">
      <Container>
        <Intro />
        <HeroPost
          title={heroPost.title}
          coverImage={heroPost.coverImage}
          date={heroPost.date}
          author={heroPost.author}
          slug={heroPost.slug}
          excerpt={heroPost.excerpt}
        />
        {morePosts.length > 0 && <MoreStories posts={morePosts} />}
      </Container>
    </main>
  );
}
