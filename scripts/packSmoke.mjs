// Release check: pack the package, install the tarball into an empty project, and use it the way a consumer
// would. ESM import, CommonJS require, TypeScript under modern and legacy resolution, the internals subpath,
// the worker thread and the physics patch command. Also fails if the tarball holds anything it should not.
//   node scripts/packSmoke.mjs [--keep]
import { execFileSync } from 'node:child_process'
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs'
import { tmpdir } from 'node:os'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = join(dirname(fileURLToPath(import.meta.url)), '..')
const pkg = JSON.parse(readFileSync(join(root, 'package.json'), 'utf8'))
const keep = process.argv.includes('--keep')
const MAX_UNPACKED_MB = 25
const FORBIDDEN = [/^bench\//, /^rust\//, /^test\//, /^docs\//, /\.run\//, /\.jar$/, /\.mca$/, /\.tgz$/, /\.env/]
const REQUIRED = ['dist/esm/index.js', 'dist/cjs/index.js', 'dist/types/index.d.ts', 'dist/cjs/index.d.ts', 'dist/esm/internals.js', 'dist/esm/worker/entry.js', 'dist/cjs/worker/entry.js', 'scripts/patchPhysics.mjs', 'README.md', 'LICENSE']

// On Windows npm is a .cmd, which only runs through a shell: quote what the shell would split.
const quote = (a) => process.platform === 'win32' && /[\s"&|<>^]/.test(a) ? `"${a.replace(/"/g, '""')}"` : a
const npm = (args, cwd) => execFileSync('npm', args.map(quote), { cwd, encoding: 'utf8', shell: process.platform === 'win32', stdio: ['ignore', 'pipe', 'pipe'] })
const node = (args, cwd) => execFileSync(process.execPath, args, { cwd, encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] })
const step = (s) => console.log(`[pack] ${s}`)

const tmp = mkdtempSync(join(tmpdir(), 'pf-pack-'))
let failed = false
try {
  step('packing')
  const parsed = JSON.parse(npm(['pack', '--ignore-scripts', '--json', '--pack-destination', tmp], root))
  const info = parsed[0]
  const paths = info.files.map(f => f.path.replace(/\\/g, '/'))

  const bad = paths.filter(p => FORBIDDEN.some(r => r.test(p)))
  if (bad.length > 0) throw new Error(`forbidden files in the tarball:\n  ${bad.slice(0, 20).join('\n  ')}`)
  const missing = REQUIRED.filter(p => !paths.includes(p))
  if (missing.length > 0) throw new Error(`missing from the tarball: ${missing.join(', ')}`)
  const mb = info.unpackedSize / 1e6
  if (mb > MAX_UNPACKED_MB) throw new Error(`unpacked ${mb.toFixed(1)} MB is over the ${MAX_UNPACKED_MB} MB limit`)
  step(`${paths.length} files, ${(info.size / 1e6).toFixed(1)} MB packed, ${mb.toFixed(1)} MB unpacked`)

  const project = join(tmp, 'consumer')
  execFileSync(process.execPath, ['-e', `require('fs').mkdirSync(${JSON.stringify(project)})`])
  writeFileSync(join(project, 'package.json'), JSON.stringify({ name: 'consumer', version: '1.0.0', private: true }))
  step('installing the tarball into an empty project')
  npm(['install', '--no-audit', '--no-fund', join(tmp, info.filename), 'mineflayer@4', 'typescript@5', '@types/node@22'], project)

  step('ESM import')
  writeFileSync(join(project, 'esm.mjs'), `
import def, { pathfinder, createPathfinder, Movements, goals, ActionError } from '${pkg.name}'
import { Solver, buildSnapshot } from '${pkg.name}/internals'
for (const [n, v] of Object.entries({ pathfinder, createPathfinder, Movements, ActionError, Solver, buildSnapshot })) {
  if (typeof v !== 'function') throw new Error(n + ' is not exported')
}
if (typeof goals.GoalBlock !== 'function' || def.pathfinder !== pathfinder) throw new Error('goals or the default export are wrong')
console.log('ok')
`)
  node(['esm.mjs'], project)

  step('CommonJS require')
  writeFileSync(join(project, 'cjs.cjs'), `
const { pathfinder, Movements, goals } = require('${pkg.name}')
const internals = require('${pkg.name}/internals')
if (typeof pathfinder !== 'function' || typeof Movements !== 'function' || typeof goals.GoalNear !== 'function') throw new Error('bad exports')
if (typeof internals.Solver !== 'function') throw new Error('bad internals')
console.log('ok')
`)
  node(['cjs.cjs'], project)

  step('TypeScript consumer')
  writeFileSync(join(project, 'consumer.ts'), `
import { createBot } from 'mineflayer'
import { Vec3 } from 'vec3'
import { pathfinder, Movements, goals, autoEatIntegration } from '${pkg.name}'
import type { GotoOptions, Pathfinder, PathfinderOptions, PathfinderEvents, PathStopReason } from '${pkg.name}'
import { Solver } from '${pkg.name}/internals'
const bot = createBot({ username: 'x' })
bot.loadPlugin(pathfinder)
bot.loadPlugin(autoEatIntegration({ startAt: 16 }))
const block = bot.blockAt(new Vec3(0, 0, 0))
if (block !== null) void bot.pathfinder.dig(block)
const moves = new Movements(bot)
moves.exclusionAreasStep.push((b) => b.name === 'farmland' ? 100 : 0)
void bot.pathfinderAutoEat?.eatNow()
const pf: Pathfinder = bot.pathfinder
const options: GotoOptions = { signal: new AbortController().signal, bestEffort: true }
pf.setMovements(new Movements(bot).applyPreset('parkour'))
void pf.goto(new goals.GoalBlock(0, 64, 0), options)
void pf.computePath(new goals.GoalBlock(0, 64, 0), { timeout: 100 }).then(r => r.path.length)
pf.minHealth = 8
const off: () => void = pf.actions.wrap('dig', next => async (block, o) => await next(block, o))
off()
bot.on('path_stop', (reason: PathStopReason) => { void reason })
bot.on('pathfinder:low_health', (health: number) => { void health })
const moving: boolean = pf.isMoving()
const pOptions: PathfinderOptions = { core: 'js', logger: { warn: () => {} } }
type Reset = PathfinderEvents['path_reset']
export { moving, pOptions, Solver }
export type { Reset }
`)
  writeFileSync(join(project, 'commonjs.cts'), `
import { createBot } from 'mineflayer'
import { pathfinder, Movements, goals } from '${pkg.name}'
import type { Pathfinder } from '${pkg.name}'
import { Solver } from '${pkg.name}/internals'
const bot = createBot({ username: 'x' })
bot.loadPlugin(pathfinder)
const pf: Pathfinder = bot.pathfinder
pf.setMovements(new Movements(bot))
void pf.goto(new goals.GoalBlock(0, 64, 0))
export { Solver }
`)
  const tsc = (resolution, module, file = 'consumer.ts') => {
    writeFileSync(join(project, 'tsconfig.json'), JSON.stringify({ compilerOptions: { target: 'ES2022', module, moduleResolution: resolution, esModuleInterop: true, strict: true, noEmit: true, skipLibCheck: true, types: ['node'] }, files: [file] }))
    node([join(project, 'node_modules/typescript/lib/tsc.js'), '-p', 'tsconfig.json'], project)
  }
  tsc('NodeNext', 'NodeNext')
  tsc('Bundler', 'ESNext')
  tsc('Node10', 'CommonJS')
  // A .cts file under Node16 resolution rejects declarations that describe ESM.
  tsc('Node16', 'Node16', 'commonjs.cts')

  step('nothing was patched on install, and the patch command works')
  const physicsFile = join(project, 'node_modules/prismarine-physics/index.js')
  if (readFileSync(physicsFile, 'utf8').includes('@bulba/pathfinder')) throw new Error('installing patched prismarine-physics')
  npm(['exec', '--no', '--', 'bulba-pathfinder-patch-physics'], project)
  const patched = readFileSync(physicsFile, 'utf8')
  if (!patched.includes('@bulba/pathfinder')) throw new Error('the patch command changed nothing')

  step('the worker thread and the WebAssembly core start from the installed package')
  writeFileSync(join(project, 'worker.mjs'), `
import { Worker } from 'node:worker_threads'
import { createRequire } from 'node:module'
import { dirname, join } from 'node:path'
const require = createRequire(import.meta.url)
const dist = dirname(require.resolve('${pkg.name}/package.json'))
const w = new Worker(join(dist, 'dist/esm/worker/entry.js'), { stderr: true })
let stderr = ''
let done = false
w.stderr.on('data', d => { stderr += d })
w.on('error', e => { console.error(e); process.exit(1) })
w.on('exit', c => { if (!done) { console.error('worker exited early', c); process.exit(1) } })
w.postMessage({ t: 'reserve', cells: 1000 })
setTimeout(async () => {
  done = true
  await w.terminate()
  if (stderr !== '') { console.error(stderr); process.exit(1) }
  console.log('ok')
}, 1000)
`)
  node(['worker.mjs'], project)

  step('an idle worker does not hold the process open')
  writeFileSync(join(project, 'idle.mjs'), `
import { SolverWorkerHost } from '${pkg.name}/internals'
new SolverWorkerHost().prewarm(1000)
setTimeout(() => {}, 1500)
`)
  execFileSync(process.execPath, ['idle.mjs'], { cwd: project, stdio: 'ignore', timeout: 20000 })
  step('all checks passed')
} catch (error) {
  failed = true
  const e = error
  console.error(`[pack] FAILED: ${e.message}`)
  if (e.stdout) console.error(String(e.stdout).slice(-2000))
  if (e.stderr) console.error(String(e.stderr).slice(-2000))
} finally {
  if (keep) console.log(`[pack] kept ${tmp}`)
  else rmSync(tmp, { recursive: true, force: true })
}
process.exit(failed ? 1 : 0)
