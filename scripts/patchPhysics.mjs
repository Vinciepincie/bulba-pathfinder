// prismarine-physics climbs the ladder, the vine and scaffolding; vanilla's `climbable` tag also holds the
// nether vines (1.16) and the cave vines (1.17), and the body climbs them the same. This patches the installed
// package's isOnLadder to know them. And from 1.21 vanilla's jumpFromGround keeps the faster of the jump and the
// body's own rise (deltaMovement.y = max(jumpPower, vy), read from the 1.21.11 server: a jump held on a slime
// rebound keeps the rebound), where prismarine sets the jump's; the patched rule follows the bot's version.
// Each patch is idempotent (its own mark) and optional: an anchor not found is a warning, never a failed
// install (the kernel follows whichever jump rule the bot runs: physics.ts probeVanillaJump). Run at install,
// and by hand after an update of the package.
//   node scripts/patchPhysics.mjs
import { createRequire } from 'node:module'
import { readFileSync, writeFileSync } from 'node:fs'

const require = createRequire(import.meta.url)
let file
try {
  file = require.resolve('prismarine-physics')
} catch {
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
writeFileSync(file, s)
console.log('prismarine-physics: patched ' + file)
