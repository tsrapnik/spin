#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Zephyr's style and static checks, run on this module only.
#
#   scripts/lint.sh [--fix] [format] [checkpatch] [sca] [tests]
#
# With no step names, every step runs. Steps:
#   format      clang-format with zephyr/.clang-format (--fix rewrites the files)
#   checkpatch  zephyr/scripts/checkpatch.pl with Zephyr's .checkpatch.conf
#   sca         GCC -fanalyzer (ZEPHYR_SCA_VARIANT=gcc) on the nucleo_c542rc build; only findings
#               in this module fail (Zephyr and the HAL have their own)
#   tests       twister on native_sim, native_sim/native/64 (ASan + UBSan) and nucleo_c542rc
#
# Build output goes to $LINT_OUT (default: ${TMPDIR:-/tmp}/spin-lint), never into the repo.
# Exits non-zero if any step fails.

set -uo pipefail

SPIN=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TOP=$(cd "$SPIN" && west topdir) || { echo "lint: not in a west workspace" >&2; exit 2; }
ZEPHYR="$TOP/zephyr"
OUT=${LINT_OUT:-${TMPDIR:-/tmp}/spin-lint}

fix=0
steps=()
for arg in "$@"; do
	case "$arg" in
	--fix) fix=1 ;;
	format | checkpatch | sca | tests) steps+=("$arg") ;;
	-h | --help) sed -n '3,16p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
	*) echo "lint: unknown argument '$arg'" >&2; exit 2 ;;
	esac
done
[ ${#steps[@]} -eq 0 ] && steps=(format checkpatch sca tests)

mapfile -t SOURCES < <(cd "$SPIN" && find include lib tests subsys drivers samples \
	-name '*.[ch]' 2>/dev/null | sort | sed "s|^|$SPIN/|")

mkdir -p "$OUT"
failed=()

step_format() {
	if [ $fix -eq 1 ]; then
		clang-format -i --style="file:$ZEPHYR/.clang-format" "${SOURCES[@]}"
	else
		clang-format --dry-run --Werror --style="file:$ZEPHYR/.clang-format" "${SOURCES[@]}"
	fi
}

step_checkpatch() {
	# checkpatch reads .checkpatch.conf from the current directory.
	(cd "$ZEPHYR" && ./scripts/checkpatch.pl --no-tree --terse -f "${SOURCES[@]}")
}

step_sca() {
	local log="$OUT/sca-gcc.log"

	west build -p always -b nucleo_c542rc "$SPIN/tests/lib/foc" -d "$OUT/sca-gcc" \
		-- -DZEPHYR_SCA_VARIANT=gcc >"$log" 2>&1 || { tail -30 "$log"; return 1; }
	if grep -E "^$SPIN/.*: warning:" "$log"; then
		return 1
	fi
	echo "no analyzer findings in $SPIN ($(grep -c ': warning:' "$log") outside it)"
}

step_tests() {
	(cd "$TOP" && west twister -T "$SPIN/tests" -p native_sim -p native_sim/native/64 \
		-p nucleo_c542rc -O "$OUT/twister" --inline-logs)
}

for s in "${steps[@]}"; do
	echo "=== $s"
	if "step_$s"; then
		echo "--- $s: ok"
	else
		echo "--- $s: FAILED"
		failed+=("$s")
	fi
done

if [ ${#failed[@]} -ne 0 ]; then
	echo "lint: failed: ${failed[*]}"
	exit 1
fi
echo "lint: all passed"
