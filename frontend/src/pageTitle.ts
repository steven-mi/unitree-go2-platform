import { formatDuration } from "./api";

const APP_TITLE = "Unitree Go2";

export function formatPageTitle(page?: string): string {
  return page ? `${APP_TITLE} - ${page}` : APP_TITLE;
}

/** Page heading for connected robot views, e.g. `Cockpit - 192.168.1.5 · 42s`. */
export function formatConnectedPageHeading(
  prefix: string,
  robotIp: string,
  durationS?: number | null,
  connected = false,
): string {
  const base = robotIp ? `${prefix} - ${robotIp}` : prefix;
  if (!connected) return base;
  const dur = durationS ?? 0;
  return `${base} · ${dur > 0 ? formatDuration(dur) : "0s"}`;
}

export function pageTitleForPath(pathname: string): string {
  if (pathname === "/") return APP_TITLE;
  if (pathname.startsWith("/cockpit")) return formatPageTitle("Cockpit");
  if (pathname.startsWith("/recordings")) return formatPageTitle("Recordings");
  if (pathname.startsWith("/settings")) return formatPageTitle("Settings");
  return APP_TITLE;
}
