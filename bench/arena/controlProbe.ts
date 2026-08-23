// Ask the SERVER what it will accept, one control combination at a time.
//
//   npm run arena:controls -- --at 46.5,141,97.7 --face 46,142,98
//
// The race says a route was lost; this says why. It drops ONE bot on an exact
// position, holds a fixed set of controls for N ticks, and reports what the
// server did with the positions prismarine-physics produced — how far the bot
// got, and how many times it was silently teleported back.
//
// It is the tool that found the 1.21.x hitbox-precision bug: sweeping the
// starting clearance against one riser turned "the server refuses positions
// prismarine-physics produces" (which had three executor rules built around
// it) into a one-line physics fix, because the same sweep with the hitbox
// nudged climbed every case with zero corrections.
//
// ⚠ After a livelock the connection stays in a sticky refusing state, so each
// case parks in spectator well away, waits for a quiet window, and only then
// switches to survival on the test block. Without that, only the FIRST case
// in a batch is trustworthy.
import { parseArgs } from 'node:util'
import type { Bot } from 'mineflayer'
import { connectBot } from './arena.js'
import { BOT_BULBA_NOHOP } from './config.js'

const { values } = parseArgs({
  options: {
    at: { type: 'string', default: '' }, // x,y,z the bot starts on (feet)
    face: { type: 'string', default: '' }, // block to face and try to reach
    ticks: { type: 'string', default: '24' },
    /** Comma-separated: walk, sprint, jump, sprintjump, back, still. */
    controls: { type: 'string', default: 'walk,sprint' },
    /** Sweep the start back from `at` along the facing line, in steps. */
    sweep: { type: 'string', default: '' }, // e.g. "0,0.05,0.1,0.2,0.3"
    /** Apply the 1.21.x hitbox nudge (what the package does on inject). */
    nudge: { type: 'boolean', default: true },
    name: { type: 'string', default: BOT_BULBA_NOHOP } // any opped racer name
  }
})

const sleep = async (ms: number): Promise<void> => await new Promise(r => setTimeout(r, ms))

function triple (text: string, flag: string): [number, number, number] {
  const p = text.split(',').map(Number)
  if (p.length !== 3 || p.some(n => !Number.isFinite(n))) throw new Error(`${flag} must be "x,y,z"`)
  return p as [number, number, number]
}

interface Style { forward: boolean, sprint: boolean, jump: boolean, back: boolean }
const STYLES: Record<string, Style> = {
  walk: { forward: true, sprint: false, jump: true, back: false },
  sprint: { forward: true, sprint: true, jump: true, back: false },
  jump: { forward: false, sprint: false, jump: true, back: false },
  forward: { forward: true, sprint: false, jump: false, back: false },
  sprintflat: { forward: true, sprint: true, jump: false, back: false },
  back: { forward: false, sprint: false, jump: false, back: true },
  still: { forward: false, sprint: false, jump: false, back: false }
}

async function main (): Promise<void> {
  if (values.at === '' || values.face === '') {
    throw new Error('--at x,y,z and --face x,y,z are both required')
  }
  const at = triple(values.at, '--at')
  const face = triple(values.face, '--face')
  const ticks = Number(values.ticks)
  const styles = values.controls.split(',').map(s => s.trim()).filter(s => s !== '')
  for (const s of styles) if (!(s in STYLES)) throw new Error(`unknown control style "${s}" (have: ${Object.keys(STYLES).join(', ')})`)
  const sweep = values.sweep === '' ? [0] : values.sweep.split(',').map(Number)

  const bot: Bot = await connectBot(values.name, 8)
  if (values.nudge) {
    const ph = (bot as unknown as { physics: { playerHalfWidth: number, playerHeight: number } }).physics
    if (ph.playerHalfWidth === 0.3) ph.playerHalfWidth = 0.30001
    if (ph.playerHeight === 1.8) ph.playerHeight = 1.80001
  }
  console.log(`probe ${values.name} online — hitbox nudge ${values.nudge ? 'ON' : 'off'}`)
  const cmd = (c: string): void => { bot.chat(`/${c}`) }
  cmd('gamerule fall_damage false')
  await sleep(400)

  let forced = 0
  bot.on('forcedMove' as never, (() => { forced++ }) as never)
  const tick = async (): Promise<void> => { await new Promise<void>(r => bot.once('physicsTick' as never, (() => r()) as never)) }

  /** Wait until the server stops correcting — see the sticky-state warning. */
  async function quiesce (): Promise<void> {
    bot.clearControlStates()
    cmd(`gamemode spectator ${values.name}`)
    await sleep(150)
    cmd(`tp ${values.name} ${at[0]} ${at[1] + 25} ${at[2]}`)
    await sleep(1000)
    let quiet = 0
    let seen = forced
    for (let i = 0; i < 200 && quiet < 20; i++) {
      await tick()
      if (forced === seen) quiet++
      else { quiet = 0; seen = forced }
    }
  }

  // Unit vector from the start toward the faced block, so a sweep backs the
  // bot off ALONG the approach rather than along an axis.
  const ux = face[0] + 0.5 - at[0]
  const uz = face[2] + 0.5 - at[2]
  const ulen = Math.hypot(ux, uz) || 1

  for (const style of styles) {
    for (const back of sweep) {
      const sx = at[0] - (ux / ulen) * back
      const sz = at[2] - (uz / ulen) * back
      await quiesce()
      const yaw = -Math.atan2(face[0] + 0.5 - sx, face[2] + 0.5 - sz) * 180 / Math.PI
      cmd(`gamemode survival ${values.name}`)
      cmd(`effect give ${values.name} minecraft:saturation 60 255 true`)
      cmd(`tp ${values.name} ${sx} ${at[1]} ${sz} ${yaw.toFixed(1)} 0`)
      await sleep(900)
      let quiet = 0
      let seen = forced
      for (let i = 0; i < 120 && quiet < 12; i++) {
        await tick()
        if (forced === seen) quiet++
        else { quiet = 0; seen = forced }
      }

      const before = forced
      const start = bot.entity.position.clone()
      const c = STYLES[style]
      bot.look(Math.atan2(-(face[0] + 0.5 - start.x), -(face[2] + 0.5 - start.z)), 0, true)
      let peak = start.y
      for (let t = 0; t < ticks; t++) {
        peak = Math.max(peak, bot.entity.position.y)
        bot.setControlState('forward', c.forward)
        bot.setControlState('back', c.back)
        bot.setControlState('sprint', c.sprint)
        bot.setControlState('jump', c.jump)
        await tick()
      }
      bot.clearControlStates()
      const end = bot.entity.position
      const reached = end.y >= face[1] - 0.01 && bot.entity.onGround
      console.log(
        `${style.padEnd(11)} back ${back.toFixed(2)}  ` +
        `${reached ? 'REACHED' : 'stuck  '}  ` +
        `moved ${end.distanceTo(start).toFixed(3)}  peakY ${peak.toFixed(2)}  ` +
        `corrections ${forced - before}`
      )
    }
  }

  console.log(`\ntotal corrections ${forced}`)
  bot.quit()
}

await main()
