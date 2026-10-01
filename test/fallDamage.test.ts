// landingHurts (src/fallDamage.ts): the server's fall accounting, against the
// landings traced on the arena's 1.21.11 server (a velocity packet or none).
import { expect } from 'chai'
import { landingHurts } from '../src/fallDamage.js'
import { simKindOf } from '../src/playerSim.js'

const on = (name: string): number => simKindOf(name, false)

describe('landingHurts', () => {
  it('counts the whole fall from 1.21.5 and rounds the damage down', () => {
    // a jump off a ledge three up (4.25), a walk off one four up (4.00): a packet each
    expect(landingHurts(4.25, 0.22, on('stone'), false, true)).to.equal(true)
    expect(landingHurts(4, 0.65, on('stone'), false, true)).to.equal(true)
    // a jump off a ledge two up (3.25), a jump down to a slab (3.75): none
    expect(landingHurts(3.25, 0.58, on('stone'), false, true)).to.equal(false)
    expect(landingHurts(3.75, 0.43, on('stone_slab'), false, true)).to.equal(false)
  })

  it('leaves the landing tick out before 1.21.5 and rounds the damage up', () => {
    expect(landingHurts(3.75, 0.43, on('stone'), false, false)).to.equal(true)
    expect(landingHurts(3.25, 0.58, on('stone'), false, false)).to.equal(false)
    expect(landingHurts(4, 0.65, on('stone'), false, false)).to.equal(true)
  })

  it('knows the blocks that break a fall', () => {
    expect(landingHurts(20, 1, on('slime_block'), false, true)).to.equal(false)
    expect(landingHurts(5, 0.7, on('slime_block'), true, true)).to.equal(true)
    expect(landingHurts(7.9, 0.9, on('hay_block'), false, true)).to.equal(false)
    expect(landingHurts(8, 0.9, on('hay_block'), false, true)).to.equal(true)
    expect(landingHurts(7.9, 0.9, on('red_bed'), false, true)).to.equal(false)
    expect(landingHurts(8, 0.9, on('red_bed'), false, true)).to.equal(true)
    expect(landingHurts(4, 0.65, on('honey_block'), false, false)).to.equal(true)
  })
})
