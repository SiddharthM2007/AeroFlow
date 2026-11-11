import { useEffect, useState } from "react";
import type { Vec3 } from "../lib/types";

const tidy = (v: number) => String(Number(v.toPrecision(6)));

interface NumberFieldProps {
  label: string;
  value: number;
  onChange: (value: number) => void;
  step?: number;
  min?: number;
  max?: number;
  integer?: boolean;
}

/** Numeric input that keeps the user's draft text and only commits valid numbers. */
export function NumberField({ label, value, onChange, step, min, max, integer }: NumberFieldProps) {
  const [text, setText] = useState(tidy(value));

  useEffect(() => {
    setText((current) => (Number(current) === value && current.trim() !== "" ? current : tidy(value)));
  }, [value]);

  const isValid = (s: string) => {
    if (s.trim() === "") return false;
    const v = Number(s);
    return (
      Number.isFinite(v) &&
      (!integer || Number.isInteger(v)) &&
      (min === undefined || v >= min) &&
      (max === undefined || v <= max)
    );
  };

  return (
    <label className="field">
      <span>{label}</span>
      <input
        type="number"
        value={text}
        step={step ?? "any"}
        min={min}
        max={max}
        className={isValid(text) ? undefined : "invalid"}
        onChange={(e) => {
          setText(e.target.value);
          if (isValid(e.target.value)) onChange(Number(e.target.value));
        }}
        onBlur={() => setText(tidy(value))}
      />
    </label>
  );
}

interface Vec3FieldProps {
  label: string;
  value: Vec3;
  onChange: (value: Vec3) => void;
  step?: number;
  min?: number;
  labels?: [string, string, string];
}

export function Vec3Field({ label, value, onChange, step, min, labels = ["x", "y", "z"] }: Vec3FieldProps) {
  return (
    <div className="vec3">
      <div className="vec3-label">{label}</div>
      <div className="vec3-inputs">
        {value.map((v, a) => (
          <NumberField
            key={a}
            label={labels[a]}
            value={v}
            step={step}
            min={min}
            onChange={(nv) => {
              const next = [...value] as Vec3;
              next[a] = nv;
              onChange(next);
            }}
          />
        ))}
      </div>
    </div>
  );
}

export function Panel({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <section className="panel">
      <h2>{title}</h2>
      {children}
    </section>
  );
}
