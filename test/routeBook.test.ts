// `!save` folds the signs it just scanned into the route book.
//
// The version this replaces dropped every route carrying the sign note and
// wrote back only what the scan found, so a scan that came up empty — signs
// not placed yet, a spectator whose signs never landed — silently emptied the
// book of every sign-defined route.
import { strict as assert } from 'node:assert'
import { mergeSignRoutes } from '../bench/arena/routes.js'
import type { Route } from '../bench/arena/routes.js'

const NOTE = 'defined by in-world !PF signs'

const route = (id: string, over: Partial<Route> = {}): Route => ({
  id,
  name: id,
  scenario: 'mixed',
  start: [0, 64, 0],
  end: [10, 64, 10],
  ...over
})

describe('route book: folding in scanned signs', () => {
  it('keeps every existing route when the scan finds nothing', () => {
    const book = [route('r01', { notes: 'hand written' }), route('simple2', { notes: NOTE })]
    assert.deepEqual(mergeSignRoutes(book, [], NOTE).map(r => r.id), ['r01', 'simple2'])
  })

  it('adds a newly signed route without touching the others', () => {
    const book = [route('r01', { notes: 'hand written' })]
    const merged = mergeSignRoutes(book, [route('tower')], NOTE)
    assert.deepEqual(merged.map(r => r.id), ['r01', 'tower'])
    assert.equal(merged[0].notes, 'hand written')
    assert.equal(merged[1].notes, NOTE)
  })

  it('moves a route whose signs moved', () => {
    const book = [route('tower', { start: [0, 64, 0], notes: NOTE })]
    const merged = mergeSignRoutes(book, [route('tower', { start: [99, 70, 99] })], NOTE)
    assert.equal(merged.length, 1)
    assert.deepEqual(merged[0].start, [99, 70, 99])
  })

  it('keeps a route defined by signs that are no longer there', () => {
    // The sign is gone, but the route is not: !drop removes it deliberately.
    const book = [route('gone', { notes: NOTE }), route('here', { notes: NOTE })]
    assert.deepEqual(mergeSignRoutes(book, [route('here')], NOTE).map(r => r.id), ['gone', 'here'])
  })

  it('keeps a hand-set tolerance when the sign does not specify one', () => {
    const book = [route('tower', { tolerance: 2, notes: NOTE })]
    assert.equal(mergeSignRoutes(book, [route('tower')], NOTE)[0].tolerance, 2)
  })

  it('lets the sign override the tolerance when it does specify one', () => {
    const book = [route('tower', { tolerance: 2, notes: NOTE })]
    assert.equal(mergeSignRoutes(book, [route('tower', { tolerance: 5 })], NOTE)[0].tolerance, 5)
  })

  it('does not mutate the book it was given', () => {
    const book = [route('r01', { notes: 'hand written' })]
    mergeSignRoutes(book, [route('r01')], NOTE)
    assert.equal(book[0].notes, 'hand written')
  })
})
