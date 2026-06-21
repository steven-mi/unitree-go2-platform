import { Link, useParams } from "react-router-dom";
import type { RecordingSession } from "../api";
import { ReplayPlayer } from "../components/ReplayPlayer";

interface RecordingPageProps {
  sessions: RecordingSession[];
  loading: boolean;
  onSessionsChange?: () => void;
}

export function RecordingPage({ sessions, loading, onSessionsChange }: RecordingPageProps) {
  const { sessionId } = useParams<{ sessionId: string }>();
  const session = sessions.find((s) => s.id === sessionId) ?? null;

  if (loading) {
    return <div className="loading">Loading recordings…</div>;
  }

  if (!sessionId || !session) {
    return (
      <div className="hero">
        <h1>Recording not found</h1>
        <p className="hero-sub">
          {sessionId ? `"${sessionId}" is not in the recordings list.` : "No session selected."}
        </p>
        <Link to="/" className="hero-link">Back to home</Link>
      </div>
    );
  }

  return <ReplayPlayer session={session} onTagsChange={() => onSessionsChange?.()} />;
}
