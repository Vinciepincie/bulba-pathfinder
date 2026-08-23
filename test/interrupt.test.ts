// The pause protocol, on its own: when a hand-over is granted, when it is
// made to wait, and who is allowed to hold the controls at the same time.
import { expect } from 'chai'
import { EventEmitter } from 'node:events'
import { InterruptController } from '../src/interrupt.js'
import type { InterruptHandle } from '../src/interrupt.js'

interface Harness {
  ctl: InterruptController
  bot: EventEmitter
  events: Array<{ name: string, args: unknown[] }>
  /** Flip to simulate the bot being mid-jump / mid-dig. */
  safe: boolean
  pauses: string[][]
  resumes: number[]
  /** One physics tick. */
  tick: () => boolean
}

function harness (startSafe = true): Harness {
  const bot = new EventEmitter()
  const events: Array<{ name: string, args: unknown[] }> = []
  for (const name of ['pathfinder:interrupt_requested', 'pathfinder:paused', 'pathfinder:resumed']) {
    bot.on(name, (...args: unknown[]) => events.push({ name, args }))
  }
  const h: Partial<Harness> = { bot, events, safe: startSafe, pauses: [], resumes: [] }
  h.ctl = new InterruptController(bot as never, {
    isSafe: () => h.safe as boolean,
    onPause: (reasons) => { (h.pauses as string[][]).push(reasons) },
    onResume: (ms) => { (h.resumes as number[]).push(ms) }
  })
  h.tick = () => (h.ctl as InterruptController).gate()
  return h as Harness
}

/** Let queued microtasks (promise resolutions) run. */
async function flush (): Promise<void> {
  await new Promise(resolve => setImmediate(resolve))
}

describe('interrupts', () => {
  it('grants on the next tick when the bot is already somewhere safe', async () => {
    const h = harness(true)
    let handle: InterruptHandle | null = null
    const pending = h.ctl.acquire('autoeat').then(x => { handle = x })

    expect(handle).to.equal(null) // never granted inside acquire() itself
    expect(h.ctl.wanted).to.equal(true)

    expect(h.tick()).to.equal(true)
    await pending

    expect(handle).to.not.equal(null)
    expect(h.ctl.paused).to.equal(true)
    expect(h.pauses).to.deep.equal([['autoeat']])
    expect(h.events.map(e => e.name)).to.deep.equal([
      'pathfinder:interrupt_requested', 'pathfinder:paused'
    ])
  })

  it('waits while the bot is airborne and grants the moment it lands', async () => {
    const h = harness(false)
    let granted = false
    const pending = h.ctl.acquire('autoeat').then(handle => { granted = true; return handle })

    for (let i = 0; i < 5; i++) expect(h.tick()).to.equal(false)
    await flush()
    expect(granted).to.equal(false)
    expect(h.pauses).to.deep.equal([])

    h.safe = true // landed
    expect(h.tick()).to.equal(true)
    await pending
    expect(granted).to.equal(true)
  })

  it('takes the controls anyway once the deadline expires', async () => {
    const h = harness(false)
    const pending = h.ctl.acquire('autoeat', { timeout: 1 })
    expect(h.tick()).to.equal(false)

    await new Promise(resolve => setTimeout(resolve, 12))
    expect(h.tick()).to.equal(true)
    const handle = await pending
    expect(handle.active).to.equal(true)
  })

  it('grants immediately with force, without waiting for a safe moment', async () => {
    const h = harness(false)
    const pending = h.ctl.acquire('emergency', { force: true })
    expect(h.tick()).to.equal(true)
    await pending
    expect(h.ctl.paused).to.equal(true)
  })

  it('resumes only when the last holder releases, and reports the pause length', async () => {
    const h = harness(true)
    const a = h.ctl.acquire('a', { exclusive: false })
    const b = h.ctl.acquire('b', { exclusive: false })
    h.tick()
    const [ha, hb] = await Promise.all([a, b])

    expect(h.ctl.holders.sort()).to.deep.equal(['a', 'b'])
    ha.release()
    expect(h.ctl.paused).to.equal(true)
    expect(h.resumes).to.have.length(0)

    hb.release()
    expect(h.ctl.paused).to.equal(false)
    expect(h.resumes).to.have.length(1)
    expect(h.resumes[0]).to.be.a('number').and.to.be.at.least(0)
  })

  it('release is idempotent', async () => {
    const h = harness(true)
    const pending = h.ctl.acquire('once')
    h.tick()
    const handle = await pending
    handle.release()
    handle.release()
    handle.release()
    expect(h.resumes).to.have.length(1)
    expect(handle.active).to.equal(false)
  })

  // The point of exclusivity: a bot has one pair of hands. An auto-eat must
  // not swap the pickaxe out from under a dig that is already holding them.
  it('queues an exclusive request behind an exclusive holder', async () => {
    const h = harness(true)
    const first = h.ctl.acquire('dig')
    h.tick()
    const digHandle = await first

    let ateEarly = false
    const second = h.ctl.acquire('autoeat').then(handle => { ateEarly = true; return handle })

    for (let i = 0; i < 5; i++) h.tick()
    await flush()
    expect(ateEarly).to.equal(false)
    expect(h.ctl.holders).to.deep.equal(['dig'])

    digHandle.release()
    const eatHandle = await second
    expect(ateEarly).to.equal(true)
    expect(eatHandle.active).to.equal(true)
    // Released and re-taken: one resume for the dig, one fresh pause.
    expect(h.pauses).to.deep.equal([['dig'], ['autoeat']])
  })

  it('the deadline does not let a request jump an exclusive holder', async () => {
    const h = harness(true)
    const first = h.ctl.acquire('dig')
    h.tick()
    const digHandle = await first

    let granted = false
    void h.ctl.acquire('autoeat', { timeout: 1 }).then(() => { granted = true })
    await new Promise(resolve => setTimeout(resolve, 15))
    for (let i = 0; i < 3; i++) h.tick()
    await flush()

    expect(granted).to.equal(false)
    digHandle.release()
    await flush()
    expect(granted).to.equal(true)
  })

  it('lets non-exclusive observers share, but not with an exclusive holder', async () => {
    const h = harness(true)
    const watcher = h.ctl.acquire('watcher', { exclusive: false })
    h.tick()
    const watcherHandle = await watcher

    const shared = h.ctl.acquire('watcher2', { exclusive: false })
    h.tick()
    await shared
    expect(h.ctl.holders).to.have.length(2)

    // An exclusive request must wait for BOTH observers to let go.
    let exclusiveGranted = false
    void h.ctl.acquire('dig').then(() => { exclusiveGranted = true })
    for (let i = 0; i < 3; i++) h.tick()
    await flush()
    expect(exclusiveGranted).to.equal(false)

    watcherHandle.release()
    ;(await shared).release()
    await flush()
    expect(exclusiveGranted).to.equal(true)
  })

  it('does not let a queue of observers starve an exclusive request (FIFO)', async () => {
    const h = harness(true)
    const dig = h.ctl.acquire('dig')
    const observer = h.ctl.acquire('observer', { exclusive: false })
    h.tick()

    const digHandle = await dig
    expect(h.ctl.holders).to.deep.equal(['dig'])

    let observed = false
    void observer.then(() => { observed = true })
    await flush()
    expect(observed).to.equal(false)

    digHandle.release()
    await observer
    expect(observed).to.equal(true)
  })

  it('rejects a pending request when its signal aborts', async () => {
    const h = harness(false)
    const controller = new AbortController()
    const pending = h.ctl.acquire('autoeat', { signal: controller.signal })
    h.tick()
    controller.abort()

    let error: Error | null = null
    await pending.catch((e: Error) => { error = e })
    expect(error).to.not.equal(null)
    expect((error as unknown as Error).name).to.equal('InterruptAborted')
    expect(h.ctl.wanted).to.equal(false)
  })

  it('rejects straight away when the signal is already aborted', async () => {
    const h = harness(true)
    const controller = new AbortController()
    controller.abort()
    let error: Error | null = null
    await h.ctl.acquire('autoeat', { signal: controller.signal }).catch((e: Error) => { error = e })
    expect((error as unknown as Error | null)?.name).to.equal('InterruptAborted')
  })

  it('run() releases even when the body throws', async () => {
    const h = harness(true)
    const ticker = setInterval(() => h.tick(), 2)
    try {
      await h.ctl.run('boom', async () => { throw new Error('nope') })
      expect.fail('should have rethrown')
    } catch (err) {
      expect((err as Error).message).to.equal('nope')
    } finally {
      clearInterval(ticker)
    }
    expect(h.ctl.paused).to.equal(false)
    expect(h.resumes).to.have.length(1)
  })
})
