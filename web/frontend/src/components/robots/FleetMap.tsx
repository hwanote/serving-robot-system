import { resolveApiUrl } from '../../config/env'
import { useFleet } from '../../state/fleetContext'
import { sortedRobots } from '../../state/fleetReducer'
import { useStoreData } from '../../state/storeDataContext'
import { EmptyState, ErrorState, LoadingState } from '../common/StateViews'
import { MapViewport } from '../map/MapViewport'
import { robotColor } from '../map/mapStyles'
import { SemanticLayer } from '../map/SemanticLayer'
import { RobotLayer } from './RobotLayer'

/** 표시 전용 지도: 저장된 Semantic + 실시간 로봇 위치 */
export function FleetMap({ highlightIds = [] }: { highlightIds?: string[] }) {
  const { rawMap, semantics, reloadRawMap } = useStoreData()
  const { state } = useFleet()
  const map = rawMap.data
  const stale = state.socket.status !== 'open'

  if (!rawMap.loaded) return <LoadingState message="지도를 불러오는 중…" />
  if (rawMap.error && !map) return <ErrorState message={rawMap.error} onRetry={() => void reloadRawMap()} />
  if (!map) return <EmptyState title="저장된 지도가 없습니다.">지도 설정 화면에서 매핑 후 지도를 저장하세요.</EmptyState>

  const doc = semantics.data
  // revision 이 다른 Semantic 은 좌표가 맞지 않을 수 있으므로 표시하지 않음
  const docMatches = doc && doc.map_revision === map.map_revision
  return (
    <div style={{ position: 'relative', flex: 1, minWidth: 0, display: 'flex' }}>
      <MapViewport
        geometry={map}
        imageUrl={resolveApiUrl(map.image_url)}
        revisionKey={map.map_revision}
        cursor="grab"
        ariaLabel="로봇 위치 지도"
        renderOverlay={(ctx) => (
          <>
            {docMatches && <SemanticLayer ctx={ctx} doc={doc} highlightIds={highlightIds} />}
            <RobotLayer ctx={ctx} robots={sortedRobots(state)} stale={stale} colorOf={robotColor} />
          </>
        )}
      />
      {doc && !docMatches && (
        <div className="map-hint" style={{ background: 'rgba(183,121,31,.92)' }}>
          Semantic Map(r{doc.map_revision})이 현재 지도(r{map.map_revision})와 달라 표시하지 않습니다.
        </div>
      )}
    </div>
  )
}
