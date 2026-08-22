// ESM variant — copied to dist/esm/runtime/moduleDir.js by build.mjs.
// Returns the directory of the compiled runtime tree so the worker entry can
// be resolved next to it (dist/esm/worker/entry.js).
import { dirname } from 'node:path'
import { fileURLToPath } from 'node:url'

export function moduleDir () {
  return dirname(dirname(fileURLToPath(import.meta.url)))
}

export const moduleFormat = 'esm'
