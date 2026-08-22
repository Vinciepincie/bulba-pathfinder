// CJS variant — copied to dist/cjs/runtime/moduleDir.js by build.mjs.
'use strict'
const { dirname } = require('node:path')

function moduleDir () {
  return dirname(__dirname)
}

module.exports = { moduleDir, moduleFormat: 'cjs' }
