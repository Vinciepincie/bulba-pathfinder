#!/usr/bin/env node
// Optional. Brings the installed prismarine-physics in line with the game in two places: nether and cave vines
// are climbable, and from 1.21 a jump keeps the faster of its own speed and the body's rise (a jump held on a
// slime rebound keeps the rebound). The pathfinder detects both at runtime and works without them.
// Each patch is idempotent, and an anchor not found is a warning, never a failure. Run it again after updating
// prismarine-physics.
//   npx bulba-pathfinder-patch-physics
import { createRequire } from 'node:module'
import { readFileSync, renameSync, writeFileSync } from 'node:fs'
import { join } from 'node:path'

/** The copy the bot runs is mineflayer's: from the project this was run in, then from this package. */
function findPhysics () {
  const froms = [join(process.cwd(), 'noop.js'), import.meta.url]
  for (const from of froms) {
    try {
      return createRequire(createRequire(from).resolve('mineflayer')).resolve('prismarine-physics')
    } catch {}
  }
  for (const from of froms) {
    try {
      return createRequire(from).resolve('prismarine-physics')
    } catch {}
  }
  return null
}

const file = findPhysics()
if (file === null) {
  console.warn('prismarine-physics: not installed, not patched')
  process.exit(0)
}
let s = readFileSync(file, 'utf8')
let changed = false
const skip = (what) => console.warn(`prismarine-physics: ${what} not where expected, not patched`)

const MARK = '// @bulba/pathfinder: the vines of the climbable tag'
if (!s.includes(MARK)) {
  const decl = "  const scaffoldingId = blocksByName.scaffolding ? blocksByName.scaffolding.id : -1 // 1.14+\n"
  const test = "    if (scaffoldingId !== -1 && block.type === scaffoldingId) { return true }\n"
  if (!s.includes(decl) || !s.includes(test)) {
    skip('isOnLadder')
  } else {
    s = s.replace(decl, decl + `  ${MARK}\n  const vineIds = new Set(['weeping_vines', 'weeping_vines_plant', 'twisting_vines', 'twisting_vines_plant', 'cave_vines', 'cave_vines_plant'].filter(n => blocksByName[n]).map(n => blocksByName[n].id))\n`)
    s = s.replace(test, test + "    if (vineIds.has(block.type)) { return true }\n")
    changed = true
  }
}

// (the first version of this patch kept the rise on every version: upgraded in place)
const MARK_JUMP_V1 = '// @bulba/pathfinder: vanilla jumpFromGround keeps the faster rise\n'
const MARK_JUMP = '// @bulba/pathfinder: jumpFromGround keeps the faster rise from 1.21 (vanilla)'
if (!s.includes(MARK_JUMP)) {
  const physics = 'function Physics (mcData, world) {\n'
  const stock = "        vel.y = Math.fround(0.42) * ((blockBelow && blockBelow.type === honeyblockId) ? physics.honeyblockJumpSpeed : 1)\n" +
    "        if (entity.jumpBoost > 0) {\n" +
    "          vel.y += 0.1 * entity.jumpBoost\n" +
    "        }\n"
  const v1 = "        vel.y = Math.max(jumpPower, vel.y)\n"
  const gated = "        vel.y = jumpKeepsRise ? Math.max(jumpPower, vel.y) : jumpPower\n"
  if (!s.includes(physics)) {
    skip('Physics()')
  } else if (s.includes(MARK_JUMP_V1) && s.includes(v1)) {
    s = s.replace(MARK_JUMP_V1, MARK_JUMP + '\n').replace(v1, gated)
    changed = true
  } else if (s.includes(stock)) {
    s = s.replace(stock,
      `        ${MARK_JUMP}\n` +
      "        let jumpPower = Math.fround(0.42) * ((blockBelow && blockBelow.type === honeyblockId) ? physics.honeyblockJumpSpeed : 1)\n" +
      "        if (entity.jumpBoost > 0) {\n" +
      "          jumpPower += 0.1 * entity.jumpBoost\n" +
      "        }\n" + gated)
    changed = true
  } else {
    skip('the jump')
  }
  if (changed && s.includes(gated)) {
    s = s.replace(physics, physics + `  ${MARK_JUMP}\n  const jumpKeepsRise = typeof mcData.isNewerOrEqualTo === 'function' && mcData.isNewerOrEqualTo('1.21')\n`)
  }
}

if (!changed) { console.log('prismarine-physics: nothing to patch'); process.exit(0) }
// A new file renamed over the old one: pnpm hard-links node_modules into a shared store, and writing in
// place would patch every project on the machine.
writeFileSync(file + '.tmp', s)
renameSync(file + '.tmp', file)
console.log('prismarine-physics: patched ' + file)
