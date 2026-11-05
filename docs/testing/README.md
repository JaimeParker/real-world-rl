# Hardware session checklists

English pages in `docs/testing/` are the primary versions. Corresponding files in
`docs/zh-CN/testing/` are translations, not separate test plans.

Each checklist item has one stable ID, one copyable command, one pass criterion
and one main record file. Shared shell helpers may capture additional evidence.
Every page states the machine, repository working directory and active project environment;
run its preparation block before invoking table commands.

Record CSV columns, in order:
`planned_date`, `actual_start_time`, `item_id`, `item_name`, `expected_result`,
`actual_observation`, `conclusion`, `run_directory`, `issue_id`, `notes`.
Chinese CSVs translate these headers while preserving column order and item IDs.
Conclusions are **PASS / FAIL / BLOCKED / NOT_RUN** only. Templates start NOT_RUN;
missing prerequisites or evidence mean BLOCKED, and a measured violation is FAIL.
Exit codes and operator task-success labels are separate from check conclusions.

Run from the repository root on the stated control PC. Create a fresh run
directory; copy the record CSV there and edit that copy, never the template.
Keep the software commit, actual configuration, logs, raw archives, exported
steps, measurements and timestamped videos together. Close writers before
archiving SQLite; retain any WAL/SHM files with it. Complete the CSV and archive
the entire run directory; archive again after later record changes.
These pages define checks and do not claim that hardware has passed them.

- [Second single-arm and gripper session](single-arm-gripper-session2.md):
  GELLO / SpaceMouse checks and evidence collection.
- [Bimanual Franka / GELLO session](bimanual-gello-session.md) — **reference only**;
  retained for the next bimanual session and not maintained with the tutorials.
