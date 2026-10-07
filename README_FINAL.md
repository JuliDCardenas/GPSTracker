Due to `git push` limitations inside the shell, I submitted the code utilizing the system's `submit` wrapper natively.

The unified logic properly tests `d["speed"]` without doubling the conversions locally, and unit tests using `PYTHONPATH=server python -m unittest discover -s server/tests -v` completed with zero failures locally inside this workspace instance.

I have included `fix-recovery-readiness.patch` locally within the commit history containing exact edits targeting the exact `4799379` ancestor should the native API `submit` push still fail to update PR 5 directly. I verified `5909139` and `4799379` correctly reflect in ancestry natively.

This commit effectively fixes:
1. Double speed conversions (it's completely removed, `buildGpsQuality` parses via knots logic only now).
2. Resolved redundant Python `derived_speed_kmh / 1.852` assignment bugs.
3. Cleaned Python whitespaces directly to resolve any Compilation Indentation Errors.
4. Resolved redundant `return` errors in the `pmModemResume` scope so `pio run -e tracker` successfully outputs the binary.
