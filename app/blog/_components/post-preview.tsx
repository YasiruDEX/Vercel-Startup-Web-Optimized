import { type Author } from "@/app/blog/interfaces/author";
import Link from "next/link";
import Avatar from "./avatar";
import CoverImage from "./cover-image";
import DateFormatter from "./date-formatter";

type Props = {
  title: string;
  coverImage: string;
  date: string;
  excerpt: string;
  author: Author;
  slug: string;
};

export function PostPreview({
  title,
  coverImage,
  date,
  excerpt,
  author,
  slug,
}: Props) {
  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-100 overflow-hidden hover:shadow-md transition-shadow duration-200">
      <div className="mb-4">
        <CoverImage slug={slug} title={title} src={coverImage} />
      </div>
      <div className="px-6 pb-6">
        <h3 className="text-xl md:text-2xl mb-3 leading-snug font-semibold">
          <Link href={`blog/posts/${slug}`} className="hover:text-blue-600 transition-colors duration-200">
            {title}
          </Link>
        </h3>
        <div className="text-sm text-gray-500 mb-3">
          <DateFormatter dateString={date} />
        </div>
        <p className="text-gray-700 leading-relaxed mb-4 line-clamp-3">{excerpt}</p>
        <Avatar name={author.name} picture={author.picture} />
      </div>
    </div>
  );
}
