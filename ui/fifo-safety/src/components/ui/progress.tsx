interface ProgressProps {
  value: number;
  className?: string;
}

export function Progress({ value, className = "" }: ProgressProps) {
  return (
    <div className={`w-full h-2 rounded-full bg-gray-800 ${className}`}>
      <div
        className="h-full bg-green-400 rounded-full transition-all"
        style={{ width: `${value}%` }}
      />
    </div>
  );
}
