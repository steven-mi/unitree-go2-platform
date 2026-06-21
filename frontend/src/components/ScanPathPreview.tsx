import { useEffect, useRef, useState } from "react";
import { fetchFloorPlan } from "../api";
import { drawScanPreview } from "./scanMapCanvas";

interface ScanPathPreviewProps {
  scanId: string;
  className?: string;
  width?: number;
  height?: number;
}

export function ScanPathPreview({
  scanId,
  className,
  width = 88,
  height = 66,
}: ScanPathPreviewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    let cancelled = false;
    setFailed(false);

    (async () => {
      try {
        const plan = await fetchFloorPlan(scanId, 0, null, undefined, null, "scan");
        if (cancelled) return;
        drawScanPreview(canvas, plan);
      } catch {
        if (!cancelled) setFailed(true);
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [scanId]);

  return (
    <canvas
      ref={canvasRef}
      className={className ?? "scan-path-preview"}
      width={width}
      height={height}
      aria-hidden={!failed}
      title={failed ? undefined : "Scan map preview"}
    />
  );
}
