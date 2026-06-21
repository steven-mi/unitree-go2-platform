/** Persistent WebSocket for low-latency teleop velocity streaming. */

export interface DriveVelocity {
  vx: number;
  vy: number;
  vyaw: number;
}

const STREAM_HZ = 50;
const STREAM_INTERVAL_MS = 1000 / STREAM_HZ;
const RECONNECT_MS = 500;

function driveWsUrl(): string {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/api/live/drive/ws`;
}

export interface DriveChannel {
  pushNow: () => void;
  close: () => void;
}

export function openDriveChannel(
  getVelocity: () => DriveVelocity,
  onError?: (message: string) => void,
): DriveChannel {
  let closed = false;
  let ws: WebSocket | null = null;
  let reconnectTimer: number | null = null;
  let streamTimer: number | null = null;

  const sendVelocity = (vel: DriveVelocity) => {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(vel));
  };

  const tick = () => {
    if (closed) return;
    sendVelocity(getVelocity());
  };

  const connect = () => {
    if (closed) return;
    ws?.close();
    ws = new WebSocket(driveWsUrl());

    ws.onopen = () => {
      tick();
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(String(event.data)) as { error?: string };
        if (data.error) onError?.(data.error);
      } catch {
        // ignore non-json
      }
    };

    ws.onclose = () => {
      ws = null;
      if (!closed) {
        reconnectTimer = window.setTimeout(connect, RECONNECT_MS);
      }
    };

    ws.onerror = () => {
      ws?.close();
    };
  };

  connect();
  streamTimer = window.setInterval(tick, STREAM_INTERVAL_MS);

  return {
    pushNow: tick,
    close: () => {
      closed = true;
      if (reconnectTimer != null) window.clearTimeout(reconnectTimer);
      if (streamTimer != null) window.clearInterval(streamTimer);
      sendVelocity({ vx: 0, vy: 0, vyaw: 0 });
      ws?.close();
      ws = null;
    },
  };
}
