import cn from "classnames";
import Link from "next/link";
import Image from "next/image";

type Props = {
  title: string;
  src: string;
  slug?: string;
};

const CoverImage = ({ title, src, slug }: Props) => {
  const image = (
    <Image
      src={src}
      alt={`Cover Image for ${title}`}
      className={cn("shadow-sm w-full rounded-lg object-cover", {
        "hover:shadow-lg transition-shadow duration-200": slug,
      })}
      width={1300}
      height={400}
      style={{ aspectRatio: '16/6' }}
    />
  );
  return (
    <div className="relative overflow-hidden rounded-lg">
      {slug ? (
        <Link href={`/blog/posts/${slug}`} aria-label={title}>
          {image}
        </Link>
      ) : (
        image
      )}
    </div>
  );
};

export default CoverImage;
