// Mock Raw Map PNG 생성기 (개발용).
// 실제 카페 지도의 최종 구조가 아니라 UI 개발을 위한 임의 레이아웃이다.
// 사용: node scripts/generate-mock-map.mjs
// 출력: src/mocks/assets/demo_store_map.png (8bit grayscale, map_server 색상 규칙: free=254, occupied=0, unknown=205)
import { mkdirSync, writeFileSync } from 'node:fs'
import { dirname, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'
import { deflateSync } from 'node:zlib'

const WIDTH = 158
const HEIGHT = 120
const RESOLUTION = 0.05
const ORIGIN = { x: -0.75, y: -0.82 }

const FREE = 254
const OCCUPIED = 0
const UNKNOWN = 205

const rect = (x0, y0, x1, y1) => (x, y) => x >= x0 && x <= x1 && y >= y0 && y <= y1
const circle = (cx, cy, r) => (x, y) => (x - cx) ** 2 + (y - cy) ** 2 <= r * r

const room = rect(-0.5, -0.6, 6.9, 4.95)
const WALL = 0.1
const obstacles = [
  // 주방 칸막이 (출입구 x 1.0~1.6)
  rect(-0.5, 3.35, 1.0, 3.45),
  rect(1.6, 3.35, 1.7, 4.95),
  // 카운터
  rect(2.2, 4.1, 3.8, 4.5),
  // 테이블
  rect(1.65, 0.65, 2.35, 1.35),
  rect(3.65, 0.65, 4.35, 1.35),
  rect(5.45, 0.65, 6.15, 1.35),
  circle(3.0, 2.6, 0.4),
  circle(5.0, 2.6, 0.4),
  // 기둥
  circle(5.9, 3.9, 0.2),
]

function cellValue(x, y) {
  if (!room(x, y)) return UNKNOWN
  const nearWall = x < -0.5 + WALL || x > 6.9 - WALL || y < -0.6 + WALL || y > 4.95 - WALL
  if (nearWall) return OCCUPIED
  return obstacles.some((hit) => hit(x, y)) ? OCCUPIED : FREE
}

// 행 v=0 이 이미지 최상단(= map y 최대). 셀 중심 좌표로 판정.
const raw = Buffer.alloc((WIDTH + 1) * HEIGHT)
for (let v = 0; v < HEIGHT; v++) {
  raw[v * (WIDTH + 1)] = 0 // filter: none
  for (let u = 0; u < WIDTH; u++) {
    const x = ORIGIN.x + (u + 0.5) * RESOLUTION
    const y = ORIGIN.y + (HEIGHT - v - 0.5) * RESOLUTION
    raw[v * (WIDTH + 1) + 1 + u] = cellValue(x, y)
  }
}

const CRC_TABLE = Array.from({ length: 256 }, (_, n) => {
  let c = n
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1
  return c >>> 0
})
function crc32(buf) {
  let c = 0xffffffff
  for (const b of buf) c = CRC_TABLE[(c ^ b) & 0xff] ^ (c >>> 8)
  return (c ^ 0xffffffff) >>> 0
}
function chunk(type, data) {
  const len = Buffer.alloc(4)
  len.writeUInt32BE(data.length)
  const body = Buffer.concat([Buffer.from(type, 'ascii'), data])
  const crc = Buffer.alloc(4)
  crc.writeUInt32BE(crc32(body))
  return Buffer.concat([len, body, crc])
}

const ihdr = Buffer.alloc(13)
ihdr.writeUInt32BE(WIDTH, 0)
ihdr.writeUInt32BE(HEIGHT, 4)
ihdr[8] = 8 // bit depth
ihdr[9] = 0 // grayscale
const png = Buffer.concat([
  Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
  chunk('IHDR', ihdr),
  chunk('IDAT', deflateSync(raw)),
  chunk('IEND', Buffer.alloc(0)),
])

const out = resolve(dirname(fileURLToPath(import.meta.url)), '../src/mocks/assets/demo_store_map.png')
mkdirSync(dirname(out), { recursive: true })
writeFileSync(out, png)
console.log(`wrote ${out} (${WIDTH}x${HEIGHT}, ${png.length} bytes)`)
