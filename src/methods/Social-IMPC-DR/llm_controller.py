"""
Phase 5 — LLM-Enhanced Negotiation Controller

Extends NegotiationController by replacing the deterministic Phase 4 rules
(Expiry Guard + ETA Switch) with a Claude LLM call that reasons over drone
state in natural language.

Setup:
    set ANTHROPIC_API_KEY=your-key-here   (Windows)

Usage:
    controller = LLMController(
        cargo_configs,
        orbit_radius=0.7, orbit_speed=0.15,
        safe_distance=1.2, nominal_speed=0.1, eta_threshold=0.15,
        llm_cache_steps=10,
        use_llm_hook=True,
        use_llm_score=False,
    )
"""

import json
import os
import urllib.request
import urllib.error
import numpy as np

from negotiation_controller import NegotiationController
from landing_pad import PAD_CENTER
from priority import priority_score

# ── Constants ─────────────────────────────────────────────────────────────────

_API_URL    = "https://api.anthropic.com/v1/messages"
_API_VER    = "2023-06-01"
_MODEL      = "claude-haiku-4-5-20251001"
_MAX_TOKENS = 120
_TIMEOUT    = 8   # seconds


class LLMController(NegotiationController):
    """Phase 5: Claude-driven negotiation on top of Phase 4 rules."""

    def __init__(self, cargo_configs=None, orbit_radius=0.7, orbit_speed=0.15,
                 safe_distance=1.2, nominal_speed=0.1, eta_threshold=0.15,
                 use_hysteresis=True,
                 llm_model=_MODEL,
                 llm_cache_steps=10,
                 use_llm_hook=True,
                 use_llm_score=False):

        super().__init__(cargo_configs, orbit_radius, orbit_speed,
                         safe_distance, nominal_speed, eta_threshold,
                         use_hysteresis=use_hysteresis)

        self._llm_model       = llm_model
        self._llm_cache_steps = llm_cache_steps
        self._use_llm_hook    = use_llm_hook
        self._use_llm_score   = use_llm_score

        # Store initial drone configs for summary
        self._initial_configs = cargo_configs or []

        # Cache — avoids an API call every single step
        self._cached_result     = None
        self._cached_step       = -(10 ** 9)
        self._cached_active_set = None

        # Decision log for evaluation
        self.llm_decision_log = []

        # Check for API key at startup
        self._api_key = os.environ.get("ANTHROPIC_API_KEY", "")
        if not self._api_key:
            print(
                "\n  [LLMController] WARNING: ANTHROPIC_API_KEY is not set.\n"
                "  LLM calls will fail and fall back to Phase 4 rules.\n"
                "  Set it with:  set ANTHROPIC_API_KEY=your-key-here\n"
            )

    # ── Phase 5-A: LLM as negotiation hook ───────────────────────────────────

    def negotiation_hook(self, agent_list, active_drones, step):
        """Ask Claude which drone should land next.

        Falls back to Phase 4 rules on any failure.
        """
        if not self._use_llm_hook or len(active_drones) <= 1:
            return super().negotiation_hook(agent_list, active_drones, step)

        active_set = frozenset(active_drones)

        # Cache check
        fresh_cache = (
            self._cached_result is not None
            and active_set == self._cached_active_set
            and step - self._cached_step < self._llm_cache_steps
        )
        if fresh_cache:
            return self._cached_result

        # Build prompt and call LLM
        prompt = self._build_prompt(agent_list, active_drones, step)
        try:
            raw_text = self._call_llm(prompt)
            result   = self._parse_response(raw_text, agent_list, active_drones)
        except Exception as exc:
            print(f"  [LLM] step {step}: fallback to Phase 4 ({exc})")
            return super().negotiation_hook(agent_list, active_drones, step)

        # Update cache
        self._cached_result     = result
        self._cached_step       = step
        self._cached_active_set = active_set

        # Log the decision
        self.llm_decision_log.append({
            "step":    step,
            "allowed": result["allowed"],
            "method":  result["method"],
            "reason":  result.get("reason", ""),
        })

        return result

    # ── Phase 5-B: LLM as priority scorer ────────────────────────────────────

    def _score_active(self, agent_list, active_drones):
        """Override priority scoring with LLM-produced scores.

        Falls back to the formula on any error.
        """
        if not self._use_llm_score:
            return super()._score_active(agent_list, active_drones)

        prompt = self._build_scoring_prompt(agent_list, active_drones)
        try:
            raw_text = self._call_llm(prompt)
            scores   = self._parse_scores(raw_text, active_drones)
            return scores
        except Exception as exc:
            print(f"  [LLM scorer] fallback to formula ({exc})")
            return super()._score_active(agent_list, active_drones)

    # ── Prompt builders ───────────────────────────────────────────────────────

    def _build_prompt(self, agent_list, active_drones, step):
        """Compose the Phase 5-A decision prompt."""
        lines = [
            "You are coordinating medical drone deliveries to a single hospital "
            "landing pad. Only one drone can land at a time.",
            "",
            f"Step: {step}",
            "Drones waiting to land:",
        ]

        for j in active_drones:
            a    = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            eta  = dist / max(self._nominal_speed, 1e-6)
            lines.append(
                f"  Drone {j}: cargo={a.cargo_type}, "
                f"acuity={a.patient_acuity}, "
                f"time_to_expiry={a.time_to_expiry:.0f} steps, "
                f"ETA={eta:.0f} steps"
            )

        lines += [
            "",
            "Which drone should land first? Consider medical urgency and expiry risk.",
            "",
            "Respond with EXACTLY this format:",
            "DRONE <index> because <one short sentence reason>",
            f"where <index> is one of: {list(active_drones)}",
        ]

        return "\n".join(lines)

    def _build_scoring_prompt(self, agent_list, active_drones):
        """Compose the Phase 5-B scoring prompt."""
        lines = [
            "You are a medical priority scoring system for drone deliveries.",
            "Score each drone's landing priority from 0.0 (lowest) to 1.0 (highest).",
            "",
            "Drones:",
        ]

        for j in active_drones:
            a    = agent_list[j]
            dist = float(np.linalg.norm(a.p - PAD_CENTER))
            eta  = dist / max(self._nominal_speed, 1e-6)
            lines.append(
                f"  Drone {j}: cargo={a.cargo_type}, "
                f"acuity={a.patient_acuity}, "
                f"time_to_expiry={a.time_to_expiry:.0f} steps, "
                f"ETA={eta:.0f} steps"
            )

        lines += [
            "",
            "Respond with EXACTLY this format (one line per drone):",
            "SCORE <index> <value>",
            "Example:  SCORE 0 0.82",
        ]

        return "\n".join(lines)

    # ── LLM call ──────────────────────────────────────────────────────────────

    def _call_llm(self, prompt):
        """Send the prompt to Claude and return the raw text response."""
        if not self._api_key:
            raise RuntimeError("ANTHROPIC_API_KEY not set")

        payload = json.dumps({
            "model":      self._llm_model,
            "max_tokens": _MAX_TOKENS,
            "messages":   [{"role": "user", "content": prompt}],
        }).encode("utf-8")

        req = urllib.request.Request(
            _API_URL,
            data=payload,
            headers={
                "Content-Type":      "application/json",
                "x-api-key":         self._api_key,
                "anthropic-version": _API_VER,
            },
            method="POST",
        )

        with urllib.request.urlopen(req, timeout=_TIMEOUT) as resp:
            data = json.loads(resp.read().decode("utf-8"))

        return data["content"][0]["text"].strip()

    # ── Response parsers ──────────────────────────────────────────────────────

    def _parse_response(self, text, agent_list, active_drones):
        """Parse 'DRONE <index> because <reason>' into a result dict.

        Raises ValueError on malformed output so the caller falls back
        to Phase 4 rules.
        """
        allowed_idx = None
        reason      = ""

        for line in text.splitlines():
            stripped = line.strip()
            if stripped.upper().startswith("DRONE"):
                parts = stripped.split(None, 2)
                if len(parts) >= 2:
                    try:
                        idx = int(parts[1])
                        if idx in active_drones:
                            allowed_idx = idx
                            reason = parts[2] if len(parts) > 2 else ""
                            break
                    except ValueError:
                        pass

        if allowed_idx is None:
            raise ValueError(f"Could not parse drone index from: {text!r}")

        yielding = {j for j in active_drones if j != allowed_idx}

        print(f"  [LLM] Drone {allowed_idx} selected — {reason}")

        return {
            "allowed":  allowed_idx,
            "yielding": yielding,
            "method":   "llm_hook",
            "scores":   {},
            "reason":   reason,
        }

    def _parse_scores(self, text, active_drones):
        """Parse 'SCORE <index> <value>' lines into {drone_idx: float}.

        Raises ValueError if any active drone is missing a score.
        """
        scores = {}
        for line in text.splitlines():
            parts = line.strip().split()
            if len(parts) == 3 and parts[0].upper() == "SCORE":
                try:
                    idx = int(parts[1])
                    val = float(parts[2])
                    if idx in active_drones:
                        scores[idx] = float(np.clip(val, 0.0, 1.0))
                except ValueError:
                    pass

        missing = [j for j in active_drones if j not in scores]
        if missing:
            raise ValueError(f"LLM did not score drones: {missing}")

        return scores

    # ── Summary ───────────────────────────────────────────────────────────────

    def print_llm_summary(self):
        """Print a summary of all LLM arbitration decisions made during the simulation."""
        print("\n" + "=" * 50)
        print("  LLM ARBITRATION SUMMARY")
        print("=" * 50)

        # Print initial drone context
        if self._initial_configs:
            print("  Initial Drone Configurations:")
            for i, cfg in enumerate(self._initial_configs):
                print(
                    f"    Drone {i}: cargo={cfg.get('cargo_type', 'unknown'):<14} "
                    f"acuity={cfg.get('patient_acuity', 'unknown'):<10} "
                    f"tte={cfg.get('time_to_expiry', '?')}"
                )
            print()

        # Print decisions
        if not self.llm_decision_log:
            print("  No LLM calls were made.")
        else:
            print(f"  Total LLM calls: {len(self.llm_decision_log)}")
            print()
            for entry in self.llm_decision_log:
                reason = entry.get("reason", "")
                print(f"  Step {entry['step']:>3}: Drone {entry['allowed']} selected")
                if reason:
                    # Wrap reason text neatly
                    print(f"           {reason}")

        print("=" * 50 + "\n")
