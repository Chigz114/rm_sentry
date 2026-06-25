# Evidence

This directory is for evidence packets from real simulation or robot runs.

Do not backfill guessed evidence. If a run was not recorded with enough detail, leave it out or mention it only in the appropriate history/decision document as an unstructured historical note.

## Packet Layout

Use this shape for new runs:

```text
docs/evidence/
  runs_index.md
  run_YYYYMMDD_HHMM_short_name/
    metadata.md
    params_snapshot.yaml
    topic_hz.txt
    tf_tree.txt
    metrics.csv
    human_observation.md
    agent_readable_summary.md
    screenshots/
```

## Rules

- One packet should correspond to one route, scenario, or test session.
- Include the git commit or `git status` state.
- Include launch commands or enough context to reproduce the run.
- Human visual observation and agent-readable metrics are both useful; keep them separate.
- Large bags, videos, and screenshots should not be committed unless intentionally managed.

## Templates

Copy from `docs/evidence/templates/` when recording a new run.
