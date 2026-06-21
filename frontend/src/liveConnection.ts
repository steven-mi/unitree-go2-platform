import { useCallback, useEffect, useRef, useState } from "react";
import {
  connectLive,
  disconnectLive,
  fetchLiveFrame,
  fetchLiveStatus,
  fetchSettings,
  type LiveStatus,
  type ReplayFrame,
} from "./api";

export interface LiveConnectionOptions {
  /** Reconnect automatically after an unexpected drop (e.g. keyboard teleop). */
  autoReconnect?: boolean;
  autoReconnectDelayMs?: number;
}

export function useLiveConnection(statusPollMs = 2000, options: LiveConnectionOptions = {}) {
  const { autoReconnect = false, autoReconnectDelayMs = 1500 } = options;
  const [robotIp, setRobotIp] = useState("");
  const [liveStatus, setLiveStatus] = useState<LiveStatus | null>(null);
  const [connecting, setConnecting] = useState(false);
  const [disconnecting, setDisconnecting] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [frame, setFrame] = useState<ReplayFrame | null>(null);
  const connectGenRef = useRef(0);
  const userDisconnectRef = useRef(false);
  const prevConnectedRef = useRef(false);

  const connectToRobot = useCallback(async () => {
    const ip = robotIp.trim();
    if (!ip) {
      setError("Set a robot IP address in Settings");
      return;
    }

    const gen = ++connectGenRef.current;
    userDisconnectRef.current = false;
    setConnecting(true);
    setError(null);
    try {
      const current = await fetchLiveStatus();
      if (gen !== connectGenRef.current) return;
      setLiveStatus(current);

      if (current.connected && current.robot_ip === ip) {
        return;
      }

      const status = await connectLive(ip);
      if (gen !== connectGenRef.current) return;

      setLiveStatus(status);
      if (status.robot_ip) setRobotIp(status.robot_ip);
      if (status.state === "error") {
        setError(status.error ?? "Failed to connect to robot");
      }
    } catch (err) {
      if (gen !== connectGenRef.current) return;
      setError(err instanceof Error ? err.message : "Failed to connect");
    } finally {
      if (gen === connectGenRef.current) setConnecting(false);
    }
  }, [robotIp]);

  useEffect(() => {
    let cancelled = false;

    const boot = async () => {
      try {
        const [settings, status] = await Promise.all([fetchSettings(), fetchLiveStatus()]);
        if (cancelled) return;
        setRobotIp(settings.robot_ip);
        setLiveStatus(status);
      } catch (err) {
        if (!cancelled) {
          setError(err instanceof Error ? err.message : "Failed to load live status");
        }
      }
    };

    void boot();

    // Do not disconnect on tab switch / bfcache — that caused spurious drops while
    // using Point & Go or Joystick in another tab. Idle timeout closes stale sessions.

    const poll = window.setInterval(() => {
      fetchLiveStatus()
        .then((status) => {
          if (!cancelled) setLiveStatus(status);
        })
        .catch(() => {});
    }, statusPollMs);

    return () => {
      cancelled = true;
      connectGenRef.current += 1;
      window.clearInterval(poll);
    };
  }, [statusPollMs]);

  useEffect(() => {
    const wasConnected = prevConnectedRef.current;
    const isConnected = Boolean(liveStatus?.connected);
    prevConnectedRef.current = isConnected;

    if (!autoReconnect || !wasConnected || isConnected) return;
    if (userDisconnectRef.current) return;
    if (!robotIp.trim()) return;
    if (connecting || disconnecting) return;

    const timer = window.setTimeout(() => {
      setError("Connection lost — reconnecting…");
      void connectToRobot();
    }, autoReconnectDelayMs);

    return () => window.clearTimeout(timer);
  }, [
    autoReconnect,
    autoReconnectDelayMs,
    connecting,
    connectToRobot,
    disconnecting,
    liveStatus?.connected,
    robotIp,
  ]);

  useEffect(() => {
    if (!liveStatus?.connected) {
      setFrame(null);
      return;
    }

    let cancelled = false;
    let syncing = false;

    const syncStatus = async (fromFrameError = false) => {
      if (cancelled || syncing) return;
      syncing = true;
      try {
        const status = await fetchLiveStatus();
        if (!cancelled) {
          setLiveStatus(status);
          if (!status.connected) {
            setFrame(null);
            if (fromFrameError && !autoReconnect) {
              setError("Lost connection to robot — click Connect to reconnect");
            }
          }
        }
      } catch {
        if (!cancelled) {
          setLiveStatus((prev) =>
            prev ? { ...prev, connected: false, state: "idle" as const } : prev,
          );
          setFrame(null);
          if (fromFrameError && !autoReconnect) {
            setError("Lost connection to robot — click Connect to reconnect");
          }
        }
      } finally {
        syncing = false;
      }
    };

    const pollFrame = () => {
      fetchLiveFrame()
        .then((f) => {
          if (!cancelled) setFrame(f);
        })
        .catch((err: Error & { status?: number }) => {
          if (cancelled) return;
          if (err.status === 503) void syncStatus(true);
        });
    };

    pollFrame();
    const poll = window.setInterval(pollFrame, 250);

    return () => {
      cancelled = true;
      window.clearInterval(poll);
    };
  }, [autoReconnect, liveStatus?.connected]);

  const handleDisconnect = useCallback(async () => {
    connectGenRef.current += 1;
    userDisconnectRef.current = true;
    setConnecting(false);
    setDisconnecting(true);
    setError(null);
    try {
      const status = await disconnectLive();
      setLiveStatus(status);
      setFrame(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to disconnect");
    } finally {
      setDisconnecting(false);
    }
  }, []);

  const handleConnectionToggle = useCallback(async () => {
    if (liveStatus?.connected) {
      await handleDisconnect();
      return;
    }

    if (connecting || liveStatus?.state === "connecting") {
      await handleDisconnect();
      return;
    }

    await connectToRobot();
  }, [connectToRobot, connecting, handleDisconnect, liveStatus?.connected, liveStatus?.state]);

  const refreshStatus = useCallback(async () => {
    try {
      const status = await fetchLiveStatus();
      setLiveStatus(status);
    } catch {
      /* ignore */
    }
  }, []);

  const liveConnected = Boolean(liveStatus?.connected);

  return {
    robotIp,
    liveStatus,
    connecting,
    disconnecting,
    error,
    setError,
    frame,
    liveConnected,
    handleConnectionToggle,
    refreshStatus,
  };
}
