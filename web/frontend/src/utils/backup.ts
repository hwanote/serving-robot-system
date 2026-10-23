/** 백업 id(20261023-153012) → "2026-10-23 15:30:12" . 모르는 형태면 id 그대로 */
export function formatBackupId(id: string): string {
  const m = /^(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})(?:-(\d+))?$/.exec(id)
  if (!m) return id
  return `${m[1]}-${m[2]}-${m[3]} ${m[4]}:${m[5]}:${m[6]}${m[7] ? ` (${m[7]})` : ''}`
}

const pad = (n: number) => String(n).padStart(2, '0')

/** 백업 시각을 사용자 PC 시간대 기준 "2026-10-23 15:30:12" 로. 읽을 수 없으면 id 로 대신한다 */
export function formatBackupTime(b: { id: string; created_at: string }): string {
  const d = new Date(b.created_at)
  if (Number.isNaN(d.getTime())) return formatBackupId(b.id)
  const suffix = /^\d{8}-\d{6}-(\d+)$/.exec(b.id)?.[1]
  return `${d.getFullYear()}-${pad(d.getMonth() + 1)}-${pad(d.getDate())} ${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}${suffix ? ` (${suffix})` : ''}`
}
