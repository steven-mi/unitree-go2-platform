import { useCallback, useEffect, useState } from "react";
import { Link, Navigate, Route, Routes, useLocation } from "react-router-dom";
import { Menu } from "lucide-react";
import { fetchRecordings, type RecordingSession } from "./api";
import { Sidebar } from "./components/Sidebar";
import { AppFooter } from "./components/AppFooter";
import { Go2Icon } from "./components/Go2Icon";
import { HomePage } from "./pages/HomePage";
import { CockpitPage } from "./pages/CockpitPage";
import { RecordingPage } from "./pages/RecordingPage";
import { RecordingsHomePage } from "./pages/RecordingsHomePage";
import { SettingsPage } from "./pages/SettingsPage";
import { pageTitleForPath } from "./pageTitle";

export default function App() {
  const location = useLocation();
  const [sessions, setSessions] = useState<RecordingSession[]>([]);
  const [sessionsLoading, setSessionsLoading] = useState(true);
  const [mobileNavOpen, setMobileNavOpen] = useState(false);

  const refreshSessions = useCallback(() => {
    fetchRecordings().then(setSessions);
  }, []);

  useEffect(() => {
    if (!location.pathname.startsWith("/cockpit")) {
      document.title = pageTitleForPath(location.pathname);
    }
  }, [location.pathname]);

  useEffect(() => {
    setMobileNavOpen(false);
  }, [location.pathname]);

  useEffect(() => {
    fetchRecordings()
      .then(setSessions)
      .finally(() => setSessionsLoading(false));
  }, []);

  return (
    <div className="app">
      <Sidebar
        sessions={sessions}
        sessionsLoading={sessionsLoading}
        onSessionsChange={refreshSessions}
        mobileOpen={mobileNavOpen}
        onMobileClose={() => setMobileNavOpen(false)}
      />
      <div
        className={`sidebar-backdrop${mobileNavOpen ? " visible" : ""}`}
        onClick={() => setMobileNavOpen(false)}
        aria-hidden="true"
      />

      <main className="main">
        <header className="mobile-topbar">
          <button
            type="button"
            className="mobile-nav-toggle"
            onClick={() => setMobileNavOpen(true)}
            aria-label="Open navigation menu"
          >
            <Menu size={22} strokeWidth={1.75} />
          </button>
          <Link to="/" className="mobile-topbar-brand">
            <Go2Icon size={24} />
            <span>Unitree Go2</span>
          </Link>
        </header>
        <div className="main-content">
          <Routes>
            <Route
              path="/cockpit"
              element={<CockpitPage onSessionsChange={refreshSessions} />}
            />
            <Route path="/live" element={<Navigate to="/cockpit" replace />} />
            <Route path="/joystick" element={<Navigate to="/cockpit" replace />} />
            <Route path="/scan/:scanId" element={<Navigate to="/cockpit" replace />} />
            <Route path="/scan" element={<Navigate to="/cockpit" replace />} />
            <Route path="/point-go/:scanId" element={<Navigate to="/cockpit" replace />} />
            <Route path="/point-go" element={<Navigate to="/cockpit" replace />} />
            <Route path="/" element={<HomePage />} />
            <Route path="/settings" element={<SettingsPage />} />
            <Route path="/recordings" element={<RecordingsHomePage sessions={sessions} loading={sessionsLoading} />} />
            <Route
              path="/recordings/:sessionId"
              element={
                <RecordingPage
                  sessions={sessions}
                  loading={sessionsLoading}
                  onSessionsChange={refreshSessions}
                />
              }
            />
          </Routes>
        </div>
        <AppFooter />
      </main>
    </div>
  );
}
