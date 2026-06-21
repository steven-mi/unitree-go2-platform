import { Film, Radio } from "lucide-react";
import { Link } from "react-router-dom";
import { Go2Icon } from "../components/Go2Icon";

const FEATURES = [
  {
    to: "/cockpit",
    icon: Radio,
    title: "Cockpit",
    description: "Connect, build a live floor plan, drive with the keyboard, and point-and-go the dog to destinations on the map.",
  },
  {
    to: "/recordings",
    icon: Film,
    title: "Recordings",
    description: "Replay saved sessions — scrub video, lidar, and floor plan timelines.",
  },
] as const;

export function HomePage() {
  return (
    <div className="home">
      <div className="home-header">
        <Go2Icon className="home-logo" size={72} />
        <h1>Welcome to Unitree Go2 Dashboard</h1>
        <p className="home-sub">
          Open-source browser dashboard for the Unitree Go2. Connect over WebRTC on your local
          network to drive, map rooms, plan routes, and replay sessions.
        </p>
      </div>
      <div className="feature-grid">
        {FEATURES.map(({ to, icon: Icon, title, description }) => (
          <Link key={title} to={to} className="feature-card">
            <div className="feature-card-icon">
              <Icon size={22} strokeWidth={1.75} />
            </div>
            <div className="feature-card-body">
              <h2>{title}</h2>
              <p>{description}</p>
            </div>
          </Link>
        ))}
      </div>
    </div>
  );
}
