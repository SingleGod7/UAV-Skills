#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import os
import re
import sqlite3
import sys
import time
from pathlib import Path

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

READ_ACTIONS = {
    sqlite3.SQLITE_SELECT,
    sqlite3.SQLITE_READ,
    sqlite3.SQLITE_FUNCTION,
    sqlite3.SQLITE_RECURSIVE,
}


def emit(payload, code=0):
    print(json.dumps(payload, ensure_ascii=False))
    raise SystemExit(code)


def find_database(name):
    configured = os.environ.get("UAV_DATA_DIR")
    if configured:
        candidate = Path(configured) / name
        if candidate.is_file():
            return candidate
        emit({"status": "ERROR", "message": f"cannot locate {name} in UAV_DATA_DIR"}, 1)
    here = Path(__file__).resolve()
    for parent in (here.parent, *here.parents):
        for candidate in (parent / name, parent / "data" / name):
            if candidate.is_file():
                return candidate
    emit({"status": "ERROR", "message": f"cannot locate {name}"}, 1)


def authorizer(action, _arg1, _arg2, _database, _trigger):
    return sqlite3.SQLITE_OK if action in READ_ACTIONS else sqlite3.SQLITE_DENY


def run(database_name, table_name):
    parser = argparse.ArgumentParser(
        description=f"Run one read-only SQLite query against {database_name} ({table_name})"
    )
    parser.add_argument("--sql", help="one SELECT or WITH ... SELECT; omitted reads stdin")
    args = parser.parse_args()

    sql = args.sql if args.sql is not None else sys.stdin.read()
    if not sql.strip():
        emit({"status": "INVALID_ARGUMENT", "message": "SQL cannot be empty"}, 2)
    if not re.match(r"^\s*(SELECT|WITH)\b", sql, re.IGNORECASE):
        emit({"status": "INVALID_ARGUMENT", "message": "only SELECT or WITH ... SELECT is allowed"}, 2)

    database = find_database(database_name)
    uri = database.resolve().as_uri() + "?mode=ro"
    connection = sqlite3.connect(uri, uri=True)
    connection.row_factory = sqlite3.Row
    deadline = time.monotonic() + 3.0
    connection.execute("PRAGMA query_only=ON")
    connection.set_authorizer(authorizer)
    connection.set_progress_handler(lambda: 1 if time.monotonic() > deadline else 0, 1000)

    try:
        cursor = connection.execute(sql)
        rows = cursor.fetchmany(101)
        truncated = len(rows) > 100
        rows = rows[:100]
        data = [dict(row) for row in rows]
        status = "OK" if data else "NOT_FOUND"
        emit({"status": status, "count": len(data), "truncated": truncated, "data": data})
    except sqlite3.Error as exc:
        emit({"status": "INVALID_ARGUMENT", "message": str(exc)}, 2)
    finally:
        connection.close()
