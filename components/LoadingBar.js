// components/LoadingBar.js
export function LoadingBar({ progress }) {
  return (
    <div className="relative w-full h-2 bg-gray-200 mt-4">
      <div
        className="absolute top-0 left-0 h-full bg-black"
        style={{ width: `${progress}%`, transition: 'width 0.1s ease-in-out' }}
      ></div>
    </div>
  );
}
