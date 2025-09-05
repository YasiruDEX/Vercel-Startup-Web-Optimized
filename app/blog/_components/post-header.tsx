import Avatar from "./avatar";
import CoverImage from "./cover-image";
import DateFormatter from "./date-formatter";
import { PostTitle } from "@/app/blog/_components/post-title";
import { type Author } from "@/app/blog/interfaces/author";

type Props = {
  title: string;
  coverImage: string;
  date: string;
  author: Author;
};

export function PostHeader({ title, coverImage, date, author }: Props) {
  return (
    <>
      <PostTitle>{title}</PostTitle>
      <div className="hidden md:block md:mb-8">
        <Avatar name={author.name} picture={author.picture} />
      </div>
      <div className="mb-8 md:mb-12">
        <CoverImage title={title} src={coverImage} />
      </div>
      <div className="max-w-4xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="block md:hidden mb-6">
          <Avatar name={author.name} picture={author.picture} />
        </div>
        <div className="mb-8 text-lg text-gray-600">
          <DateFormatter dateString={date} />
        </div>
      </div>
    </>
  );
}
