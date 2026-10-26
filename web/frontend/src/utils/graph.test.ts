import { describe, expect, it } from 'vitest'
import { compassOf, stopSideText } from './graph'

describe('compassOf', () => {
  it('0°=동, 90°=북, 180°·-180°=서, -90°=남', () => {
    expect(compassOf(0)).toBe('동')
    expect(compassOf(90)).toBe('북')
    expect(compassOf(180)).toBe('서')
    expect(compassOf(-180)).toBe('서')
    expect(compassOf(-90)).toBe('남')
    expect(compassOf(270)).toBe('남')
  })
  it('허용 오차 안이면 그 방향, 밖이면 null', () => {
    expect(compassOf(95)).toBe('북')
    expect(compassOf(-84)).toBe('남')
    expect(compassOf(45)).toBeNull()
  })
})

describe('stopSideText', () => {
  it('테이블·주방 등은 대상의 반대쪽에 서서 바라보는 방향을 말한다', () => {
    expect(stopSideText({ kind: 'table_service', yaw_deg: -90 })).toBe('대상의 북쪽에 서서 남쪽을 바라봄')
    expect(stopSideText({ kind: 'pickup', yaw_deg: 90 })).toBe('대상의 남쪽에 서서 북쪽을 바라봄')
  })
  it('대기 위치는 바라보는 방향만 말한다', () => {
    expect(stopSideText({ kind: 'standby', yaw_deg: 0 })).toBe('동쪽을 바라봄')
  })
  it('축 방향이 아니면 각도로 말한다', () => {
    expect(stopSideText({ kind: 'table_service', yaw_deg: 45 })).toBe('45° 방향을 바라봄')
  })
})
