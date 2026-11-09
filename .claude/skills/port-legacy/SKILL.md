---
name: port-legacy
description: Port useful logic from old Src/ or Include/ code into its owning service
argument-hint: <path under Src/ or Include/>
user-invocable: true
context: fork
agent: planner
---
Legacy file: $ARGUMENTS

Read the file and the AGENTS.md service table. List what is worth porting and to which service, and
flag anything that breaks a rule (for example automatic motion). If it touches an open question, start
with `STOP: ask <owner>` and end.

Otherwise end by printing a plan in the format `/feature` consumes (the planner output format, every
item tagged `[fast]` or `[careful]`), headed by one line the user can paste:
`/feature <one-line task naming the legacy file and target service>`. Do not run anything else.
