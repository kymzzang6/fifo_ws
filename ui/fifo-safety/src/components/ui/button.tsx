import type { ReactNode } from "react";

interface ButtonProps {
  children: ReactNode;
  className?: string;
  onClick?: () => void;
}

export function Button({ children, className = "", onClick }: ButtonProps) {
  return (
    <button
      onClick={onClick}
      className={`px-4 py-2 rounded-lg font-semibold bg-yellow-400 text-black hover:bg-yellow-300 transition ${className}`}
    >
      {children}
    </button>
  );
}
