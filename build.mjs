// Build script: ESM + CJS + type declarations, plus the two hand-written
// format-specific runtime shims (module-dir resolution can't be expressed in
// one TS source that compiles to both formats — import.meta is a syntax error
// under CommonJS output, __dirname doesn't exist under ESM).
import { execFileSync } from 'node:child_process'
import { cpSync, existsSync, mkdirSync, rmSync, writeFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = dirname(fileURLToPath(import.meta.url))

// Resolve tsc from the dependency tree directly (no npx: it would try to
// DOWNLOAD typescript when devDependencies are absent). When typescript is
// unavailable but a dist tree already exists (e.g. a pre-built checkout),
// keep it and exit cleanly instead of failing the install.
let tscPath = null
try {
  tscPath = createRequire(import.meta.url).resolve('typescript/lib/tsc.js')
} catch {
  if (existsSync(join(root, 'dist/esm/index.js'))) {
    console.warn('[build] typescript unavailable — keeping the existing dist/ tree')
    process.exit(0)
  }
  console.error('[build] typescript is not installed and no dist/ exists — install devDependencies first')
  process.exit(1)
}
const run = (project) => execFileSync(process.execPath, [tscPath, '-p', project], { cwd: root, stdio: 'inherit' })

rmSync(join(root, 'dist'), { recursive: true, force: true })

run('tsconfig.esm.json')
run('tsconfig.cjs.json')
run('tsconfig.types.json')

// Nested package.json markers so Node treats each tree by its own format
// regardless of the package root's "type": "module".
writeFileSync(join(root, 'dist/esm/package.json'), JSON.stringify({ type: 'module' }, null, 2) + '\n')
writeFileSync(join(root, 'dist/cjs/package.json'), JSON.stringify({ type: 'commonjs' }, null, 2) + '\n')

// Format-specific moduleDir shims (see src/runtime/). tsc never compiles
// these; they are copied verbatim into the matching output tree.
for (const [variant, out] of [['moduleDir.esm.js', 'esm'], ['moduleDir.cjs.js', 'cjs']]) {
  mkdirSync(join(root, `dist/${out}/runtime`), { recursive: true })
  cpSync(join(root, 'src/runtime', variant), join(root, `dist/${out}/runtime/moduleDir.js`))
}
mkdirSync(join(root, 'dist/types/runtime'), { recursive: true })
cpSync(join(root, 'src/runtime/moduleDir.d.ts'), join(root, 'dist/types/runtime/moduleDir.d.ts'))

console.log('[build] dist/esm + dist/cjs + dist/types written')
