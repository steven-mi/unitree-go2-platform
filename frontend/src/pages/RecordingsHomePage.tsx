import { Link, Navigate } from "react-router-dom";
import type { RecordingSession } from "../api";

interface RecordingsHomePageProps {
  sessions: RecordingSession[];
  loading: boolean;
}

export function RecordingsHomePage({ sessions, loading }: RecordingsHomePageProps) {
  if (loading) {
    return <div className="loading">Loading recordings…</div>;
  }

  const latest = sessions[0];
  if (latest) {
    return <Navigate to={`/recordings/${latest.id}`} replace />;
  }

  return (
    <div className="hero">
      <h1>No recordings yet</h1>
      <p className="hero-sub">
        Go to Cockpit, connect to the dog, and press Record.
      </p>
      <Link to="/cockpit" className="hero-link">Open Cockpit</Link>
    </div>
  );
}
