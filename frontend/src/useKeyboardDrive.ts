import { useEffect, useRef, useState } from "react";
import { openDriveChannel } from "./driveChannel";
import { stopDrive } from "./api";

const DEFAULT_VX = 0.55;
const DEFAULT_VY = 0.25;
const DEFAULT_VYAW = 0.65;

const MOVEMENT_KEYS = new Set(["w", "a", "s", "d", "arrowleft", "arrowright"]);

function isEditableTarget(target: EventTarget | null): boolean {
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName;
  return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || target.isContentEditable;
}

function computeVelocity(keys: Set<string>): { vx: number; vy: number; vyaw: number } {
  let vx = 0;
  let vy = 0;
  let vyaw = 0;

  if (keys.has("w")) vx += DEFAULT_VX;
  if (keys.has("s")) vx -= DEFAULT_VX;
  if (keys.has("a")) vy += DEFAULT_VY;
  if (keys.has("d")) vy -= DEFAULT_VY;
  if (keys.has("arrowleft")) vyaw += DEFAULT_VYAW;
  if (keys.has("arrowright")) vyaw -= DEFAULT_VYAW;

  return { vx, vy, vyaw };
}

/** Fire-and-forget stop when the WebSocket channel is torn down. */
function stopDriveHttp(): void {
  void stopDrive().catch(() => {});
}

export function useKeyboardDrive(
  connected: boolean,
  enabled: boolean,
  onError?: (message: string) => void,
) {
  const keysRef = useRef(new Set<string>());
  const [pressedKeys, setPressedKeys] = useState<ReadonlySet<string>>(() => new Set());
  const onErrorRef = useRef(onError);
  const pushRef = useRef<(() => void) | null>(null);
  onErrorRef.current = onError;

  const syncPressedKeys = () => setPressedKeys(new Set(keysRef.current));

  useEffect(() => {
    if (!enabled) {
      keysRef.current.clear();
      setPressedKeys(new Set());
    }
  }, [enabled]);

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      if (!enabled) return;
      if (isEditableTarget(e.target)) return;
      const key = e.key.toLowerCase();
      if (!MOVEMENT_KEYS.has(key)) return;
      e.preventDefault();
      if (keysRef.current.has(key)) return;
      keysRef.current.add(key);
      syncPressedKeys();
      pushRef.current?.();
    };

    const onKeyUp = (e: KeyboardEvent) => {
      if (!enabled) return;
      const key = e.key.toLowerCase();
      if (!MOVEMENT_KEYS.has(key)) return;
      keysRef.current.delete(key);
      syncPressedKeys();
      pushRef.current?.();
    };

    const onBlur = () => {
      if (!enabled) return;
      keysRef.current.clear();
      syncPressedKeys();
      pushRef.current?.();
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    window.addEventListener("blur", onBlur);

    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
      window.removeEventListener("blur", onBlur);
      keysRef.current.clear();
      setPressedKeys(new Set());
    };
  }, [enabled]);

  useEffect(() => {
    if (!connected || !enabled) {
      pushRef.current = null;
      if (!enabled) {
        keysRef.current.clear();
        setPressedKeys(new Set());
      }
      return;
    }

    const channel = openDriveChannel(
      () => computeVelocity(keysRef.current),
      (message) => onErrorRef.current?.(message),
    );
    pushRef.current = channel.pushNow;
    channel.pushNow();

    return () => {
      pushRef.current = null;
      channel.close();
      stopDriveHttp();
    };
  }, [connected, enabled]);

  return { pressedKeys };
}
