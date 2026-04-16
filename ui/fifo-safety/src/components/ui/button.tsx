import type { ReactNode, ButtonHTMLAttributes } from "react";

interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  children: ReactNode;
  className?: string;
  onClick?: () => void;
}

export function Button({ 
  children, 
  className = "",
  ...props
 }: ButtonProps) {
  return (
    <button
      {...props}
      className={`px-4 py-2 rounded-lg font-semibold bg-yellow-400 text-black hover:bg-yellow-300 transition ${className}`}
    >
      {children}
    </button>
  );
}
