interface Go2IconProps {
  className?: string;
  size?: number;
}

export function Go2Icon({ className = "", size = 28 }: Go2IconProps) {
  return (
    <img
      src="/go2-icon.png"
      alt=""
      className={className ? `go2-icon ${className}` : "go2-icon"}
      width={size}
      height={size}
      draggable={false}
    />
  );
}
