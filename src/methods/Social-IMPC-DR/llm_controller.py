"""
Phase 5 — LLM-Enhanced Negotiation Controller (SCAFFOLD — NOT IMPLEMENTED)

Extends NegotiationController by replacing the deterministic Phase 4 rules
(Expiry Guard + ETA Switch) with an LLM call that reasons over drone state
in natural language. Designed as an ablation study against Phase 4 to
measure what each decision component contributes.

See README.md "Phase 5 — LLM-Enhanced Negotiation (Planned)" for the full
ablation matrix (5-A: LLM hook, 5-B: LLM scorer, 5-C: both).

This file is a scaffold only — every override raises NotImplementedError.
A teammate is expected to fill in:
  - LLM client setup (OpenAI / local model / etc.)
  - Prompt construction in `_build_prompt()`
  - Response parsing in `_parse_response()`
  - Caching / fallback logic in `negotiation_hook()`

Usage (once implemented):
    controller = LLMController(
        cargo_configs,
        orbit_radius=0.7, orbit_speed=0.15, safe_distance=1.2,
        nominal_speed=0.1, eta_threshold=0.15,
        llm_model="gpt-4o-mini",
        llm_cache_steps=10,
        use_llm_hook=True,        # Phase 5-A
        use_llm_score=False,      # Phase 5-B
    )
"""

import numpy as np
from negotiation_controller import NegotiationController
from landing_pad import PAD_CENTER


class LLMController(NegotiationController):
    """Phase 5: LLM-driven negotiation. SCAFFOLD — implementation pending."""

    def __init__(self, cargo_configs=None, orbit_radius=0.7, orbit_speed=0.15,
                 safe_distance=1.2, nominal_speed=0.1, eta_threshold=0.15,
                 use_hysteresis=True,
                 llm_model="gpt-4o-mini",
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

        # Cache: avoid an LLM call every step. Re-use last decision for
        # `llm_cache_steps` steps unless the active set changes.
        self._cached_result      = None
        self._cached_step        = -10**9
        self._cached_active_set  = None

        # TODO(teammate): instantiate LLM client here.
        # e.g. self._client = OpenAI(api_key=os.environ["OPENAI_API_KEY"])
        self._client = None

    # ------------------------------------------------------------------
    # Phase 5-A — LLM as negotiation_hook
    # ------------------------------------------------------------------
    def negotiation_hook(self, agent_list, active_drones, step):
        """Ask the LLM which drone should land next.

        Falls back to Phase 4 rules (super().negotiation_hook) on:
          - LLM disabled (`use_llm_hook=False`)
          - timeout / API error
          - unparsable response
          - drone index out of range
        """
        if not self._use_llm_hook or len(active_drones) <= 1:
            return super().negotiation_hook(agent_list, active_drones, step)

        # TODO(teammate): cache check, prompt build, LLM call, parse, fallback.
        # Suggested skeleton:
        #
        #   active_set = frozenset(active_drones)
        #   fresh_cache = (
        #       self._cached_result is not None
        #       and active_set == self._cached_active_set
        #       and step - self._cached_step < self._llm_cache_steps
        #   )
        #   if fresh_cache:
        #       return self._cached_result
        #
        #   prompt = self._build_prompt(agent_list, active_drones, step)
        #   try:
        #       text = self._call_llm(prompt)
        #       result = self._parse_response(text, active_drones)
        #   except Exception as e:
        #       print(f"  [LLM] {step}: fallback to Phase 4 rules ({e})")
        #       return super().negotiation_hook(agent_list, active_drones, step)
        #
        #   self._cached_result     = result
        #   self._cached_step       = step
        #   self._cached_active_set = active_set
        #   return result

        raise NotImplementedError(
            "LLMController.negotiation_hook: Phase 5-A not implemented. "
            "Set use_llm_hook=False to fall back to Phase 4 rules."
        )

    # ------------------------------------------------------------------
    # Phase 5-B — LLM as priority scorer
    # ------------------------------------------------------------------
    def _score_active(self, agent_list, active_drones):
        """Override priority scoring with an LLM-produced ranking.

        When `use_llm_score=False`, defers to PriorityManager's formula.
        """
        if not self._use_llm_score:
            return super()._score_active(agent_list, active_drones)

        # TODO(teammate): build a ranking prompt, call LLM, parse to
        # {drone_idx: score_in_[0,1]}. Fall back to formula on any error.
        raise NotImplementedError(
            "LLMController._score_active: Phase 5-B not implemented. "
            "Set use_llm_score=False to fall back to the formula."
        )

    # ------------------------------------------------------------------
    # Helpers (stubs)
    # ------------------------------------------------------------------
    def _build_prompt(self, agent_list, active_drones, step):
        """Compose the LLM prompt from current drone state.

        Suggested template (see README Phase 5 section for a worked example):

            You are coordinating medical drone deliveries to a single
            landing pad. Only one drone can land at a time. Choose which
            drone should land next.

            Step: {step}
            Drones:
              Drone {idx}: cargo={cargo}, acuity={acuity},
                           distance={dist:.2f}m, ETA={eta:.0f} steps,
                           time_to_expiry={tte:.0f}, score={score:.3f}
              ...

            Respond with exactly: DRONE <index> because <brief reason>
        """
        raise NotImplementedError("Prompt construction is teammate's task.")

    def _call_llm(self, prompt):
        """Send the prompt to the LLM and return the raw text response."""
        raise NotImplementedError("LLM client call is teammate's task.")

    def _parse_response(self, text, active_drones):
        """Parse `DRONE <index> because <reason>` into a result dict.

        Must return the same schema as NegotiationController hooks:
            {
                "allowed":  int,
                "yielding": set[int],
                "method":   "llm_hook",
                "scores":   dict[int, float],   # may be empty for 5-A
            }
        Raise ValueError on malformed output so the caller can fall back.
        """
        raise NotImplementedError("Response parsing is teammate's task.")
