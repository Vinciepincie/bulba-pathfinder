// Proves an edit touched only comments: each TypeScript file, compiled with comments stripped, must
// produce byte-identical output on both sides.
//   node scripts/checkCommentsOnly.mjs <baseDir> <editedDir> [file ...]
// With no files, every src/**/*.ts present in editedDir is compared.
import { createRequire } from 'node:module'
import { existsSync, readdirSync, readFileSync, statSync } from 'node:fs'
import { join, relative } from 'node:path'

const require = createRequire(import.meta.url)
const ts = require('typescript')

const [base, edited, ...named] = process.argv.slice(2)
if (base === undefined || edited === undefined) {
  console.error('usage: node scripts/checkCommentsOnly.mjs <baseDir> <editedDir> [file ...]')
  process.exit(2)
}

function walk (dir, out = []) {
  for (const name of readdirSync(dir)) {
    const p = join(dir, name)
    if (statSync(p).isDirectory()) walk(p, out)
    else if (p.endsWith('.ts') && !p.endsWith('.d.ts')) out.push(p)
  }
  return out
}

const compile = (file) => ts.transpileModule(readFileSync(file, 'utf8'), {
  fileName: file,
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.ESNext, removeComments: true, isolatedModules: true }
}).outputText

const files = named.length > 0 ? named : walk(join(edited, 'src')).map(p => relative(edited, p).replace(/\\/g, '/'))
let bad = 0
for (const f of files) {
  const a = join(base, f)
  const b = join(edited, f)
  if (!existsSync(a) || !existsSync(b)) { console.log(`MISSING  ${f}`); bad++; continue }
  if (compile(a) !== compile(b)) { console.log(`CODE CHANGED  ${f}`); bad++ }
}
console.log(bad === 0 ? `ok: ${files.length} files differ in comments at most` : `${bad} of ${files.length} files failed`)
process.exit(bad === 0 ? 0 : 1)
