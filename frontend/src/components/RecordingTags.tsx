import { useState } from "react";
import { Plus, X } from "lucide-react";
import { updateRecordingTags } from "../api";
import { tagColorIndex } from "../tagColors";

interface RecordingTagsProps {
  sessionId: string;
  tags: string[];
  onChange: (tags: string[]) => void;
}

export function RecordingTags({ sessionId, tags, onChange }: RecordingTagsProps) {
  const [draft, setDraft] = useState("");
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const persist = async (next: string[]) => {
    setSaving(true);
    setError(null);
    try {
      const saved = await updateRecordingTags(sessionId, next);
      onChange(saved);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to save tags");
    } finally {
      setSaving(false);
    }
  };

  const addTag = () => {
    const value = draft.trim();
    if (!value) return;
    if (tags.some((tag) => tag.toLowerCase() === value.toLowerCase())) {
      setDraft("");
      return;
    }
    setDraft("");
    void persist([...tags, value]);
  };

  const removeTag = (tag: string) => {
    void persist(tags.filter((item) => item !== tag));
  };

  return (
    <div className="recording-tags">
      <div className="recording-tags-row">
        {tags.map((tag) => (
          <span key={tag} className="recording-tag tag-chip" data-color={tagColorIndex(tag)}>
            {tag}
            <button
              type="button"
              className="recording-tag-remove"
              onClick={() => void removeTag(tag)}
              disabled={saving}
              aria-label={`Remove tag ${tag}`}
            >
              <X size={12} strokeWidth={2} />
            </button>
          </span>
        ))}
        <div className="recording-tags-input">
          <input
            type="text"
            className="recording-tag-field"
            value={draft}
            onChange={(e) => setDraft(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") {
                e.preventDefault();
                addTag();
              }
            }}
            placeholder="Add tag"
            spellCheck={false}
            disabled={saving}
          />
          <button
            type="button"
            className="recording-tag-add"
            onClick={() => addTag()}
            disabled={saving || !draft.trim()}
            aria-label="Add tag"
          >
            <Plus size={14} strokeWidth={2} />
          </button>
        </div>
      </div>
      {error && <div className="recording-tags-error">{error}</div>}
    </div>
  );
}
