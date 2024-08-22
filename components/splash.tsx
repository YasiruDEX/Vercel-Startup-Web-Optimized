
export function SplashScreen() {
  return (
    <div className="flex items-center justify-center min-h-screen">
      <div className="space-y-4 text-center">
        <h1 className="text-4xl font-bold tracking-tighter text-foreground sm:text-5xl md:text-6xl">
          Aura Digital Labs
        </h1>
        <div className="flex items-center justify-center">
          <div className="h-1 w-16 rounded-full bg-primary animate-pulse" />
        </div>
      </div>
    </div>
  )
}
