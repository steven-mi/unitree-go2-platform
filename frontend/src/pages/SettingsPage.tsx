import { useCallback, useEffect, useState } from "react";
import { Trash2 } from "lucide-react";
import {
  deleteScan,
  fetchScan,
  fetchScans,
  fetchSettings,
  formatScanLabel,
  restoreLatestScan,
  saveSettings,
  type AppSettings,
  type ScanSession,
} from "../api";
import { ScanPathPreview } from "../components/ScanPathPreview";

function formatScanDate(iso?: string): string {
  if (!iso) return "—";
  const d = new Date(iso);
  if (Number.isNaN(d.getTime())) return iso;
  return d.toLocaleString();
}

function hasMapData(scan: ScanSession): boolean {
  return (scan.scan_count ?? 0) > 0 || (scan.lidar_count ?? 0) > 0;
}

export function SettingsPage() {
  const [settings, setSettings] = useState<AppSettings | null>(null);
  const [robotIp, setRobotIp] = useState("");
  const [aesKey, setAesKey] = useState("");
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [saved, setSaved] = useState(false);

  const [latestScan, setLatestScan] = useState<ScanSession | null>(null);
  const [historicalScans, setHistoricalScans] = useState<ScanSession[]>([]);
  const [scansLoading, setScansLoading] = useState(true);
  const [restoringId, setRestoringId] = useState<string | null>(null);
  const [deletingId, setDeletingId] = useState<string | null>(null);
  const [scanMessage, setScanMessage] = useState<string | null>(null);
  const [scanError, setScanError] = useState<string | null>(null);

  const refreshScans = useCallback(async () => {
    setScansLoading(true);
    try {
      const [latest, historical] = await Promise.all([fetchScan("latest"), fetchScans()]);
      setLatestScan(latest);
      setHistoricalScans(historical);
    } catch {
      setLatestScan(null);
      setHistoricalScans([]);
    } finally {
      setScansLoading(false);
    }
  }, []);

  useEffect(() => {
    let cancelled = false;
    setLoading(true);
    fetchSettings()
      .then((data) => {
        if (cancelled) return;
        setSettings(data);
        setRobotIp(data.robot_ip);
        setAesKey(data.aes_128_key ?? "");
      })
      .catch((err) => {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : "Failed to load settings");
        }
      })
      .finally(() => {
        if (!cancelled) setLoading(false);
      });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    void refreshScans();
  }, [refreshScans]);

  const dirty =
    settings != null &&
    (robotIp.trim() !== settings.robot_ip || (aesKey.trim() || null) !== (settings.aes_128_key || null));

  const handleSave = useCallback(async () => {
    setSaving(true);
    setError(null);
    setSaved(false);
    try {
      const next = await saveSettings({
        robot_ip: robotIp.trim(),
        aes_128_key: aesKey.trim() || null,
      });
      setSettings(next);
      setRobotIp(next.robot_ip);
      setAesKey(next.aes_128_key ?? "");
      setSaved(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to save settings");
    } finally {
      setSaving(false);
    }
  }, [aesKey, robotIp]);

  const handleRestore = useCallback(
    async (scan: ScanSession) => {
      const label = formatScanLabel(scan);
      if (
        !window.confirm(
          `Restore "${label}" as the active map? The current map will be archived with today's date.`,
        )
      ) {
        return;
      }

      setRestoringId(scan.id);
      setScanMessage(null);
      setScanError(null);
      try {
        const result = await restoreLatestScan(scan.id);
        setLatestScan(result.latest);
        await refreshScans();
        const archived = result.archived_id ? ` Previous map saved as ${result.archived_id}.` : "";
        setScanMessage(`Restored ${label} to the active map.${archived}`);
      } catch (err) {
        setScanError(err instanceof Error ? err.message : "Failed to restore scan");
      } finally {
        setRestoringId(null);
      }
    },
    [refreshScans],
  );

  const handleDelete = useCallback(
    async (scan: ScanSession) => {
      const label = formatScanLabel(scan);
      if (!window.confirm(`Delete "${label}"? This cannot be undone.`)) return;

      setDeletingId(scan.id);
      setScanMessage(null);
      setScanError(null);
      try {
        await deleteScan(scan.id);
        await refreshScans();
        setScanMessage(`Deleted ${label}.`);
      } catch (err) {
        setScanError(err instanceof Error ? err.message : "Failed to delete scan");
      } finally {
        setDeletingId(null);
      }
    },
    [refreshScans],
  );

  const scanBusy = restoringId != null || deletingId != null;

  return (
    <div className="settings-page">
      <header className="settings-header">
        <div>
          <h2>Settings</h2>
          <p className="settings-subtitle">
            Configure how the dashboard connects to your Go2 on the local network.
          </p>
        </div>
      </header>

      <section className="settings-panel">
        <h3>Robot connection</h3>
        <p className="settings-help">
          Saved to <code>config.yml</code> in the project root. Used as the default robot the
          Cockpit connects to.
        </p>

        {loading ? (
          <p className="settings-muted">Loading…</p>
        ) : (
          <form
            className="settings-form"
            onSubmit={(e) => {
              e.preventDefault();
              void handleSave();
            }}
          >
            <label className="settings-field">
              <span className="settings-label">Robot IP address</span>
              <input
                className="settings-input"
                type="text"
                inputMode="decimal"
                autoComplete="off"
                spellCheck={false}
                placeholder="0.0.0.0"
                value={robotIp}
                onChange={(e) => {
                  setRobotIp(e.target.value);
                  setSaved(false);
                }}
              />
            </label>

            <label className="settings-field">
              <span className="settings-label">AES-128 key</span>
              <input
                className="settings-input settings-input-mono"
                type="text"
                autoComplete="off"
                spellCheck={false}
                placeholder="32 hex characters (optional on older firmware)"
                value={aesKey}
                onChange={(e) => {
                  setAesKey(e.target.value);
                  setSaved(false);
                }}
              />
            </label>
            <p className="settings-help">
              Required on Go2 firmware ≥ 1.1.15. Fetch once per robot with{" "}
              <code>unitree-fetch-aes-key</code>. Leave blank on older firmware.
            </p>

            {dirty && <p className="settings-muted">Save to apply for new connections.</p>}

            {error && <p className="settings-error">{error}</p>}
            {saved && !error && <p className="settings-success">Settings saved.</p>}

            <button type="submit" className="settings-save-btn" disabled={saving || !robotIp.trim()}>
              {saving ? "Saving…" : "Save"}
            </button>
          </form>
        )}
      </section>

      <section className="settings-panel">
        <h3>Maps &amp; scans</h3>
        <p className="settings-help">
          Cockpit continuously saves the active map to <code>scans/latest</code>. Review archived
          maps below, then restore one to make it the active map in the Cockpit.
        </p>

        {scansLoading ? (
          <p className="settings-muted">Loading scans…</p>
        ) : (
          <>
            {latestScan && hasMapData(latestScan) && (
              <div className="settings-scan-card settings-scan-card-active">
                <ScanPathPreview
                  scanId="latest"
                  className="settings-scan-preview"
                  width={240}
                  height={180}
                />
                <div className="settings-scan-card-body">
                  <div className="settings-scan-active-label">Active map</div>
                  <div className="settings-scan-item-title">Latest</div>
                  <div className="settings-scan-item-meta">
                    {latestScan.lidar_count ?? 0} lidar
                    {(latestScan.scan_count ?? 0) > 0 ? ` · ${latestScan.scan_count} ingested` : ""}
                    {latestScan.updated_at ? ` · updated ${formatScanDate(latestScan.updated_at)}` : ""}
                  </div>
                </div>
              </div>
            )}

            {historicalScans.length === 0 ? (
              <p className="settings-muted">No archived maps yet. Use Reset scan in Cockpit to archive the current map.</p>
            ) : (
              <ul className="settings-scan-list">
                {historicalScans.map((scan) => (
                  <li key={scan.id} className="settings-scan-card">
                    {hasMapData(scan) ? (
                      <ScanPathPreview
                        scanId={scan.id}
                        className="settings-scan-preview"
                        width={240}
                        height={180}
                      />
                    ) : (
                      <div className="settings-scan-preview settings-scan-preview-empty">
                        No floor plan
                      </div>
                    )}
                    <div className="settings-scan-card-body">
                      <div className="settings-scan-item-title">{formatScanLabel(scan)}</div>
                      <div className="settings-scan-item-meta">
                        {scan.lidar_count ?? 0} lidar
                        {(scan.scan_count ?? 0) > 0 ? ` · ${scan.scan_count} ingested` : ""}
                        {scan.archived_at
                          ? ` · archived ${formatScanDate(scan.archived_at)}`
                          : scan.created_at
                            ? ` · ${formatScanDate(scan.created_at)}`
                            : ""}
                      </div>
                      <div className="settings-scan-card-actions">
                        <button
                          type="button"
                          className="settings-restore-btn"
                          disabled={scanBusy || !hasMapData(scan)}
                          onClick={() => void handleRestore(scan)}
                        >
                          {restoringId === scan.id ? "Restoring…" : "Restore"}
                        </button>
                        <button
                          type="button"
                          className="settings-delete-btn"
                          disabled={scanBusy}
                          onClick={() => void handleDelete(scan)}
                          aria-label={`Delete ${formatScanLabel(scan)}`}
                        >
                          <Trash2 size={14} strokeWidth={1.75} />
                          {deletingId === scan.id ? "Deleting…" : "Delete"}
                        </button>
                      </div>
                    </div>
                  </li>
                ))}
              </ul>
            )}

            {scanError && <p className="settings-error">{scanError}</p>}
            {scanMessage && <p className="settings-success">{scanMessage}</p>}
          </>
        )}
      </section>
    </div>
  );
}
