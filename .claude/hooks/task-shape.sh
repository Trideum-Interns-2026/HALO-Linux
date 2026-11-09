#!/usr/bin/env bash
# TaskCreated. Exit 2 unless the subject is "<impl|test|verify|safety|docs>: <path or all>".
subject=$(python3 -c 'import json,sys; print((json.load(sys.stdin).get("task_subject") or "").strip())')
if [[ "$subject" =~ ^(impl|test|verify|safety|docs):\ [^[:space:]].* ]]; then
    exit 0
fi
echo "Task subject must be '<impl|test|verify|safety|docs>: <path or all>', e.g. 'impl: Movement/src/node.cpp' or 'verify: all'. Got: '$subject'" >&2
exit 2
