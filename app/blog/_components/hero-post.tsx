import Avatar from "@/app/blog/_components/avatar";
import CoverImage from "@/app/blog/_components/cover-image";
import { type Author } from "@/app/blog/interfaces/author";
import Link from "next/link";
import DateFormatter from "./date-formatter";

type Props = {
  title: string;
  coverImage: string;
  date: string;
  excerpt: string;
  author: Author;
  slug: string;
};

export function HeroPost({
  title,
  coverImage,
  date,
  excerpt,
  author,
  slug,
}: Props) {
  return (
    <section className="mb-16 md:mb-20">
      <div className="mb-6 md:mb-8">
        <CoverImage title={title} src={coverImage} slug={slug} />
      </div>
      <div className="md:grid md:grid-cols-2 md:gap-x-12 lg:gap-x-16">
        <div>
          <h3 className="mb-4 text-3xl lg:text-4xl xl:text-5xl leading-tight font-bold">
            <Link href={`blog/posts/${slug}`} className="hover:underline">
              {title}
            </Link>
          </h3>
          <div className="mb-4 md:mb-0 text-lg text-gray-600">
            <DateFormatter dateString={date} />
          </div>
        </div>
        <div>
          <p className="text-lg leading-relaxed mb-4 text-gray-700">{excerpt}</p>
          <Avatar name={author.name} picture={author.picture} />
        </div>
      </div>
    </section>
  );
}
