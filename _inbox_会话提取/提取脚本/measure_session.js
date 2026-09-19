/* 量一个 DSH 会话文件的真实体积构成。
 * 用途：判断"这个会话到底多大、成本花在哪"，给"该不该换新会话"提供数字依据。
 *
 * 用法：
 *   node --max-old-space-size=6144 measure_session.js <session.v3.jsonl.zstd>
 *
 * ⚠️ DSH 的会话文件是**多帧拼接**的 zstd（追加写）。
 *    Node 的 zstdDecompressSync 只解第一帧就停，所以必须自己走帧边界。
 *    帧边界逻辑与 `zstd_walk.js` 相同（解析块头求精确帧尾），这里内联一份，
 *    免得依赖两个文件。
 *
 * 只读，不改任何东西。
 */
const fs = require('fs')
const zlib = require('zlib')

const path = process.argv[2]
if (!path) {
  console.error('用法: node measure_session.js <session.v3.jsonl.zstd>')
  process.exit(2)
}

const buf = fs.readFileSync(path)
const MAGIC = 0xfd2fb528

/* 从 s 开始的一帧占多少字节；帧头非法时返回 null。 */
function frameLength(s) {
  if (s + 6 > buf.length) return null
  if (buf.readUInt32LE(s) !== MAGIC) return null
  const desc = buf[s + 4]
  const fcsFlag = desc >> 6
  const singleSegment = (desc >> 5) & 1
  const hasChecksum = (desc >> 2) & 1
  const didFlag = desc & 3
  let off = s + 5
  if (!singleSegment) off += 1
  if (didFlag === 1) off += 1
  else if (didFlag === 2) off += 2
  else if (didFlag === 3) off += 4
  const fcsSize = fcsFlag === 0 ? (singleSegment ? 1 : 0) : fcsFlag === 1 ? 2 : fcsFlag === 2 ? 4 : 8
  off += fcsSize
  for (let guard = 0; guard < 1e6; guard++) {
    if (off + 3 > buf.length) return null
    const bh = buf[off] | (buf[off + 1] << 8) | (buf[off + 2] << 16)
    off += 3
    const last = bh & 1
    const btype = (bh >> 1) & 3
    const bsize = bh >> 3
    if (btype === 0) off += bsize
    else if (btype === 1) off += 1
    else if (btype === 2) off += bsize
    else return null
    if (last) break
  }
  if (hasChecksum) off += 4
  return off - s
}

const parts = []
let pos = 0
let frames = 0
while (pos < buf.length) {
  if (buf.readUInt32LE(pos) !== MAGIC) { pos += 1; continue }
  const len = frameLength(pos)
  if (!len || len <= 0) { pos += 1; continue }
  try {
    parts.push(zlib.zstdDecompressSync(buf.subarray(pos, pos + len)))
    frames++
  } catch (e) { /* 跳过坏帧 */ }
  pos += len
}

const all = Buffer.concat(parts)
const text = all.toString('utf8')
const lines = text.split('\n').filter((l) => l.trim() !== '')

console.log('压缩后      ' + (buf.length / 1048576).toFixed(1) + ' MB   (' + frames + ' 帧)')
console.log('解压后      ' + (all.length / 1048576).toFixed(1) + ' MB')
console.log('记录数      ' + lines.length)

for (let i = 0; i < 2; i++) {
  try { console.log('第' + i + '行字段  ' + Object.keys(JSON.parse(lines[i])).join(',')) } catch (e) { }
}

/* 归类：type 优先，取不到用 message.role。 */
const byType = new Map()
const rows = []
for (let i = 0; i < lines.length; i++) {
  const L = lines[i].length
  let type = '?'
  let role = ''
  try {
    const o = JSON.parse(lines[i])
    type = o.type || o.kind || '?'
    role = (o.message && o.message.role) || o.role || ''
  } catch (e) { type = 'parse-fail' }
  const key = role ? type + '/' + role : type
  const cur = byType.get(key) || { n: 0, bytes: 0 }
  cur.n++
  cur.bytes += L
  byType.set(key, cur)
  rows.push([L, i, key])
}

console.log('')
console.log('类别                    条数        字节数      占比')
for (const [k, v] of [...byType.entries()].sort((a, b) => b[1].bytes - a[1].bytes)) {
  console.log('  ' + k.padEnd(20) + String(v.n).padStart(6) +
    String((v.bytes / 1048576).toFixed(1) + ' MB').padStart(12) +
    String(((v.bytes / all.length) * 100).toFixed(1) + '%').padStart(9))
}

/* ── 对"成本"最有解释力的三项细分 ─────────────────────────────
   ① 被注入回来的对话块（StrataGate）吃掉多少 —— 它们是 user/message 里最大的几条；
   ② 压缩事件本身多大；
   ③ 有没有 token 用量字段可查（有就直接读，不用估）。 */
let sgBytes = 0
let sgCount = 0
const bigUser = []
for (const [L, i, key] of rows) {
  if (key !== 'user/message') continue
  if (lines[i].includes('StrataGate conversation block')) { sgBytes += L; sgCount++ }
  if (L > 100 * 1024) bigUser.push([L, i])
}
console.log('')
console.log('注入的对话块（StrataGate）  ' + sgCount + ' 条, ' + (sgBytes / 1048576).toFixed(1) + ' MB')
console.log('user/message 中 >100KB 的    ' + bigUser.length + ' 条, ' +
  (bigUser.reduce((s, r) => s + r[0], 0) / 1048576).toFixed(1) + ' MB')

console.log('')
console.log('压缩事件：')
for (let i = 0; i < lines.length; i++) {
  if (!lines[i].includes('"compaction/')) continue
  const t = lines[i].slice(0, 400)
  const m = t.match(/"type":"(compaction\/[a-z]+)"/)
  if (m && (m[1] === 'compaction/summary' || m[1] === 'compaction/start')) {
    console.log('  行' + String(i).padStart(6) + '  ' + (lines[i].length / 1024).toFixed(0).padStart(6) + ' KB  ' + m[1])
  }
}

console.log('')
console.log('token 用量字段探测：')
for (const key of ['inputTokens', 'outputTokens', 'promptTokens', 'totalTokens', 'cacheReadTokens', 'usage']) {
  const n = (text.match(new RegExp('"' + key + '"', 'g')) || []).length
  console.log('  ' + key.padEnd(18) + n + ' 次')
}

rows.sort((a, b) => b[0] - a[0])
console.log('')
console.log('最长的 12 条：')
for (const [L, i, key] of rows.slice(0, 12)) {
  let preview = ''
  try { preview = lines[i].replace(/\s+/g, ' ').slice(0, 100) } catch (e) { }
  console.log('  ' + String((L / 1024).toFixed(0) + ' KB').padStart(8) + '  行' + String(i).padStart(6) + '  ' + key.padEnd(20) + preview)
}
