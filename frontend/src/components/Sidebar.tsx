import { useState } from "react";
import { ChevronRight, Film, PanelLeftClose, Radio, Settings, Trash2, X } from "lucide-react";
import { Link, useLocation, useMatch, useNavigate } from "react-router-dom";
import type { RecordingSession } from "../api";
import { deleteRecording, formatDuration, formatSessionLabel } from "../api";
import { Go2Icon } from "./Go2Icon";
import { tagColorIndex } from "../tagColors";

const STORAGE_KEY = "sidebar-collapsed";

interface SidebarProps {
  sessions: RecordingSession[];
  sessionsLoading: boolean;
  onSessionsChange: () => void;
  mobileOpen: boolean;
  onMobileClose: () => void;
}

export function Sidebar({
  sessions,
  sessionsLoading,
  onSessionsChange,
  mobileOpen,
  onMobileClose,
}: SidebarProps) {
  const navigate = useNavigate();
  const location = useLocation();
  const [collapsed, setCollapsed] = useState(
    () => localStorage.getItem(STORAGE_KEY) === "true",
  );
  const [deletingId, setDeletingId] = useState<string | null>(null);
  const recordingMatch = useMatch("/recordings/:sessionId");
  const selectedRecordingId = recordingMatch?.params.sessionId ?? null;
  const onHome = location.pathname === "/";
  const onCockpit = !onHome && location.pathname.startsWith("/cockpit");
  const onRecordings = !onHome && location.pathname.startsWith("/recordings");
  const onSettings = !onHome && location.pathname.startsWith("/settings");

  const goHome = (e: React.MouseEvent) => {
    e.preventDefault();
    if (location.pathname === "/") return;
    navigate("/");
  };

  const toggle = () => {
    setCollapsed((prev) => {
      const next = !prev;
      localStorage.setItem(STORAGE_KEY, String(next));
      return next;
    });
  };

  const handleDelete = async (e: React.MouseEvent, session: RecordingSession) => {
    e.preventDefault();
    e.stopPropagation();
    if (deletingId) return;
    if (!window.confirm(`Delete recording "${formatSessionLabel(session)}"? This cannot be undone.`)) {
      return;
    }
    setDeletingId(session.id);
    try {
      await deleteRecording(session.id);
      onSessionsChange();
      if (selectedRecordingId === session.id) {
        navigate("/recordings", { replace: true });
      }
    } catch (err) {
      window.alert(err instanceof Error ? err.message : "Failed to delete recording");
    } finally {
      setDeletingId(null);
    }
  };

  return (
    <aside className={`sidebar${collapsed ? " collapsed" : ""}${mobileOpen ? " mobile-open" : ""}`}>
      <div className="sidebar-header">
        <Link
          to="/"
          className={collapsed ? "sidebar-brand-icon" : "sidebar-brand"}
          onClick={goHome}
          title="Unitree Go2"
        >
          <Go2Icon size={collapsed ? 28 : 24} />
          {!collapsed && <span className="sidebar-brand-text">Unitree Go2</span>}
        </Link>
        <button
          type="button"
          className={`sidebar-toggle${collapsed ? " sidebar-toggle-expand" : ""}`}
          onClick={toggle}
          aria-label={collapsed ? "Expand sidebar" : "Collapse sidebar"}
          title={collapsed ? "Expand sidebar" : "Collapse sidebar"}
        >
          {collapsed ? <ChevronRight size={20} strokeWidth={2} /> : <PanelLeftClose size={18} strokeWidth={1.75} />}
        </button>
        <button
          type="button"
          className="sidebar-mobile-close"
          onClick={onMobileClose}
          aria-label="Close navigation menu"
        >
          <X size={20} strokeWidth={1.75} />
        </button>
      </div>

      <nav className="sidebar-nav">
        <Link
          to="/cockpit"
          className={`nav-item${onCockpit ? " active" : ""}`}
          title={collapsed ? "Cockpit" : undefined}
        >
          <Radio size={18} strokeWidth={1.75} />
          <span className="nav-label">Cockpit</span>
        </Link>
        <Link
          to="/recordings"
          className={`nav-item${onRecordings ? " active" : ""}`}
          title={collapsed ? "Recordings" : undefined}
        >
          <Film size={18} strokeWidth={1.75} />
          <span className="nav-label">Recordings</span>
        </Link>
      </nav>

      {!collapsed && onRecordings && (
        <div className="sidebar-section">
          <div className="sidebar-section-label">Recent</div>
          <div className="session-list">
            {sessionsLoading && <div className="session-item muted">Loading…</div>}
            {!sessionsLoading && sessions.map((s) => (
              <div
                key={s.id}
                className={`session-item-row${selectedRecordingId === s.id ? " active" : ""}`}
              >
                <Link
                  to={`/recordings/${s.id}`}
                  className="session-item"
                  title={formatSessionLabel(s)}
                >
                  {formatSessionLabel(s)}
                  {(s.tags?.length ?? 0) > 0 && (
                    <div className="session-item-tags">
                      {s.tags!.map((tag) => (
                        <span key={tag} className="session-tag tag-chip" data-color={tagColorIndex(tag)}>
                          {tag}
                        </span>
                      ))}
                    </div>
                  )}
                  <div className="session-item-meta">
                    {formatDuration(s.duration_s)}
                    {s.lidar_count ? ` · ${s.lidar_count} scans` : ""}
                  </div>
                </Link>
                <button
                  type="button"
                  className="session-item-delete"
                  onClick={(e) => handleDelete(e, s)}
                  disabled={deletingId === s.id}
                  aria-label="Delete recording"
                  title="Delete recording"
                >
                  <Trash2 size={15} strokeWidth={1.75} />
                </button>
              </div>
            ))}
            {!sessionsLoading && sessions.length === 0 && (
              <div className="session-item muted">No recordings yet</div>
            )}
          </div>
        </div>
      )}

      <div className="sidebar-footer">
        <Link
          to="/settings"
          className={`nav-item${onSettings ? " active" : ""}`}
          title={collapsed ? "Settings" : undefined}
        >
          <Settings size={18} strokeWidth={1.75} />
          <span className="nav-label">Settings</span>
        </Link>
      </div>
    </aside>
  );
}
