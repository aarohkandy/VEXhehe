# PROS Shim Notes

This simulator includes a minimal PROS-like adapter for autonomous structure testing.

Current supported pattern:
- User module defines `autonomous(api)` as a generator
- `api.set_tank(left, right)` sets drivetrain commands in `[-1, 1]`
- `yield from api.delay(ms)` yields control back to simulation scheduler

This is intentionally lightweight to allow headless simulator integration before full PROS API surface coverage.
