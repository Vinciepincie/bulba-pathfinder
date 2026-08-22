// Shared constant for the host↔worker dig-table residency protocol. Both
// sides evict by insertion order at the SAME bound, so the host's record of
// what it has sent always matches what the worker still holds.
export const MAX_DIG_TABLES = 8
