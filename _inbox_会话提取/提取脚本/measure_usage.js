/* 从 DSH 会话文件抽真实 token 用量，算清"这个会话花了多少、涨在哪、压缩有没有用"。
 *
 * 用法：
 *   node --max-old-space-size=6144 measure_usage.js <session.v3.jsonl.zstd>
 *
 * ── usage 对象形状（实测，不是猜）───────────────────────────────
 *   "usage":{"inputTokens":232,"outputTokens":4787,"totalTokens":225819,
 *            "cacheReadTokens":220800,"reasoningTokens":2206}
 *   逐条验算：totalTokens = inputTokens + cacheReadTokens + outputTokens（不含 reasoning）。
 *   ⚠️ 它是**单次请求**的合计，不是会话累计 —— 所以会话总量要按请求数求和。
 *   ⚠️ 同一条 usage 在一行里会出现**两次**（消息记录 + 别处回显），
 *      所以每行只取第一个 usage 对象；全部求和会翻倍。
 *
 * 只读。
 */
const fs = require('fs')
const zlib = require('zlib')

const path = process.argv[2]
if (!path) { console.error('用法: node measure_usage.js <session.v3.jsonl.zstd>'); process.exit(2) }

const buf = fs.readFileSync(path)
const MAGIC = 0xfd2fb528
function frameLength(s) {
  if (s + 6 > buf.length) return null
  if (buf.readUInt32LE(s) !== MAGIC) return null
  const desc = buf[s + 4]
  const fcsFlag = desc >> 6, single = (desc >> 5) & 1, chk = (desc >> 2) & 1, did = desc & 3
  let off = s + 5
  if (!single) off += 1
  if (did === 1) off += 1; else if (did === 2) off += 2; else if (did === 3) off += 4
  off += fcsFlag === 0 ? (single ? 1 : 0) : fcsFlag === 1 ? 2 : fcsFlag === 2 ? 4 : 8
  for (let g = 0; g < 1e6; g++) {
    if (off + 3 > buf.length) return null
    const bh = buf[off] | (buf[off + 1] << 8) | (buf[off + 2] << 16)
    off += 3
    const last = bh & 1, bt = (bh >> 1) & 3, bs = bh >> 3
    if (bt === 0) off += bs; else if (bt === 1) off += 1; else if (bt === 2) off += bs; else return null
    if (last) break
  }
  if (chk) off += 4
  return off - s
}
const parts = []
let pos = 0
while (pos < buf.length) {
  if (buf.readUInt32LE(pos) !== MAGIC) { pos += 1; continue }
  const len = frameLength(pos)
  if (!len || len <= 0) { pos += 1; continue }
  try { parts.push(zlib.zstdDecompressSync(buf.subarray(pos, pos + len))) } catch (e) { }
  pos += len
}
const lines = Buffer.concat(parts).toString('utf8').split('\n').filter((l) => l.trim())

/* ── 每行取第一个 usage 对象 ── */
const reqs = []
for (let i = 0; i < lines.length; i++) {
  const L = lines[i]
  const at = L.indexOf('"usage":{')
  if (at < 0) continue
  const obj = L.slice(at, at + 260)
  const g = (k) => { const m = obj.match(new RegExp('"' + k + '":(\\d+)')); return m ? +m[1] : 0 }
  const inp = g('inputTokens'), out = g('outputTokens'), cache = g('cacheReadTokens'), tot = g('totalTokens')
  const turn = (L.match(/"turn":(\d+)/) || [, -1])[1]
  reqs.push({ turn: +turn, inp, out, cache, tot, line: i })
}

const S = (k) => reqs.reduce((s, r) => s + r[k], 0)
const M = (n) => (n / 1e6).toFixed(1) + ' M'

console.log('=== 会话 token 总账（' + reqs.length + ' 次 LLM 请求）===')
console.log('  新输入(未命中缓存) ' + M(S('inp')))
console.log('  缓存读             ' + M(S('cache')))
console.log('  输出               ' + M(S('out')))
console.log('  ────────────────────────────')
console.log('  ⭐ 合计计费 token   ' + M(S('tot')))
console.log('  其中缓存读占比     ' + ((S('cache') / S('tot')) * 100).toFixed(1) + '%')
console.log('  输出占比           ' + ((S('out') / S('tot')) * 100).toFixed(1) + '%')

console.log('')
console.log('=== 每次请求的上下文规模（= input + cacheRead）===')
const ctx = reqs.map((r) => r.inp + r.cache)
const srt = ctx.slice().sort((a, b) => a - b)
const pct = (p) => srt[Math.floor(srt.length * p)]
console.log('  最小 ' + (Math.min(...ctx) / 1000).toFixed(0) + 'K   中位 ' + (pct(0.5) / 1000).toFixed(0) + 'K' +
  '   p90 ' + (pct(0.9) / 1000).toFixed(0) + 'K   最大 ' + (Math.max(...ctx) / 1000).toFixed(0) + 'K')

console.log('')
console.log('=== 每轮平均上下文（看压缩到底有没有把上下文压下去）===')
const byTurn = new Map()
for (const r of reqs) {
  const cur = byTurn.get(r.turn) || { n: 0, ctx: 0, out: 0, tot: 0 }
  cur.n++; cur.ctx += r.inp + r.cache; cur.out += r.out; cur.tot += r.tot
  byTurn.set(r.turn, cur)
}
const turns = [...byTurn.entries()].sort((a, b) => a[0] - b[0])
const last10 = new Set(turns.slice(-10).map((t) => t[0]))
console.log('  轮次  请求   平均上下文   该轮输出   该轮计费')
let peak = { turn: 0, ctx: 0 }
for (const [turn, v] of turns) {
  const avg = v.ctx / v.n
  if (avg > peak.ctx) peak = { turn, ctx: avg }
  if (turn % 10 !== 0 && !last10.has(turn)) continue
  console.log('  ' + String(turn).padStart(4) + String(v.n).padStart(6) +
    String((avg / 1000).toFixed(0) + 'K').padStart(12) +
    String((v.out / 1000).toFixed(0) + 'K').padStart(11) +
    String((v.tot / 1e6).toFixed(2) + 'M').padStart(11))
}
console.log('  ⭐ 上下文峰值：第 ' + peak.turn + ' 轮，平均 ' + (peak.ctx / 1000).toFixed(0) + 'K')

console.log('')
console.log('=== 压缩前后对比（看压缩收益）===')
for (let i = 0; i < lines.length; i++) {
  if (!lines[i].includes('"compaction/start"')) continue
  const before = reqs.filter((r) => r.line < i).slice(-1)[0]
  const after = reqs.filter((r) => r.line > i)[0]
  if (!before || !after) continue
  const b = before.inp + before.cache, a = after.inp + after.cache
  console.log('  行' + String(i).padStart(6) + '  压缩前 ' + (b / 1000).toFixed(0) + 'K  ->  压缩后 ' +
    (a / 1000).toFixed(0) + 'K   降幅 ' + (((b - a) / b) * 100).toFixed(0) + '%')
}

console.log('')
console.log('=== 如果现在开新会话，每轮大约省多少 ===')
const recent = turns.slice(-10)
const recentAvg = recent.reduce((s, t) => s + t[1].ctx / t[1].n, 0) / recent.length
const fresh = 20000 /* 新会话起步：系统提示 + 技能目录 + 少量文件，约 2 万 token */
console.log('  最近 10 轮平均上下文  ' + (recentAvg / 1000).toFixed(0) + 'K')
console.log('  新会话起步估计        ' + (fresh / 1000).toFixed(0) + 'K')
console.log('  ⭐ 每轮输入省约       ' + (recentAvg / fresh).toFixed(1) + ' 倍')
