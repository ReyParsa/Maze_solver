#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")"/.. && pwd)"
cd "$root_dir"

echo "== Python syntax check (py_compile) =="
python3 - <<'PY'
import pathlib, py_compile, sys
errors=[]
for p in pathlib.Path('src').rglob('*.py'):
    try:
        py_compile.compile(str(p), doraise=True)
    except Exception as e:
        errors.append((p,e))
if errors:
    print('FAIL: syntax errors:')
    for p,e in errors:
        print(f'  {p}: {e}')
    sys.exit(1)
print('PASS: no syntax errors')
PY

echo "\n== Whitespace audit (tabs in .py) =="
if grep -I -R $'\t' src/**/*.py >/dev/null 2>&1; then
  echo 'FAIL: tabs found in python files:'
  grep -nI -R $'\t' src/**/*.py || true
  exit 1
else
  echo 'PASS: no tabs found'
fi

echo "\n== Trailing whitespace audit =="
if grep -RIn "[[:blank:]]$" src/**/*.py >/dev/null 2>&1; then
  echo 'WARN: trailing whitespace present:'
  grep -RIn "[[:blank:]]$" src/**/*.py || true
else
  echo 'PASS: no trailing whitespace'
fi

echo "\nAll checks complete"
