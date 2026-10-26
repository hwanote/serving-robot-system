import { Link } from 'react-router-dom'

export function NotFoundPage() {
  return (
    <div className="page">
      <div className="card state-box">
        <strong>페이지를 찾을 수 없습니다.</strong>
        <Link to="/">대시보드로 이동</Link>
      </div>
    </div>
  )
}
